// XCP bridge: exposes ROBO TX Controllers to CANape (or any XCP client) via Vector's XCPlite, in online mode or
// through a relay TX (hybrid mode).
//
// Per controller ("tx" in online mode; "relay" and "model" with a relay), with <name> as prefix:
//   - Measurements "<name>": universal inputs I1..I8, counters C1..C4, display buttons       -> event "tx_cycle"
//   - Calibration segment "<name>_outputs": motor duties M1..M4 and single outputs O1..O8
//   - Calibration segment "<name>_config":  input modes (voltage/resistance/ultrasonic, digital/analog),
//                                           motor vs. single-output mode per M, counter edge per C
// plus measurements of the link: cycle rate and, with a relay, command round trip, resends and lost value frames.
//
// Everything is tunable live in CANape; the bridge turns changes into outputs and configuration of the controllers.
// The A2L file is generated at runtime and CANape can upload it via XCP, so nothing has to be written by hand.
// Usage: ftrobotx_xcp_bridge [-p PORT] [-a MODEL_ADDRESS]; then connect CANape to XCP on Ethernet, UDP port 5555.
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <a2l.hpp>
#include <xcplib.hpp>

#include "Cli.hpp"
#include "TxIo.hpp"

#define DESCRIPTION                                                                                                    \
    "Bridges ROBO TX Controllers to CANape via XCP on Ethernet (UDP 5555). Without -a, the controller on PORT in\n"    \
    "online mode (A2L prefix tx_). With -a, PORT is a relay TX and the model TX at that Bluetooth address runs\n"      \
    "RelayModel (hybrid/relay/tx); both are exposed (prefixes relay_ and model_)."

namespace
{

using namespace ftrobotx;

constexpr uint8_t kServerAddr[] = {0, 0, 0, 0}; // listen on all interfaces
constexpr uint16_t kServerPort = 5555;
constexpr uint32_t kQueueSize = 64 * 1024;
constexpr std::size_t kMaxControllers = 2;

volatile std::sig_atomic_t stopRequested = 0;
void onSignal(int /*signal*/)
{
    stopRequested = 1;
}

// ---------------------------------------------------------------------------------------------------------------
// Calibration segment 1: output values

struct TxOutputs
{
    int16_t m1, m2, m3, m4;                 // motor duty -512..512 (used when the motor pair is in motor mode)
    int16_t o1, o2, o3, o4, o5, o6, o7, o8; // single output duty 0..512 (used when the pair is in output mode)

    [[nodiscard]] int16_t motor(std::size_t m) const
    {
        const int16_t v[4] = {m1, m2, m3, m4};
        return v[m];
    }
    [[nodiscard]] int16_t output(std::size_t o) const
    {
        const int16_t v[8] = {o1, o2, o3, o4, o5, o6, o7, o8};
        return v[o];
    }
};

const TxOutputs kDefaultOutputs = {};

// ---------------------------------------------------------------------------------------------------------------
// Calibration segment 2: IO configuration. Nested typedefs, so CANape shows model_config.i1.mode etc.

struct InputCfg
{
    uint8_t mode;    // ftrobotx::InputMode: 0 Voltage, 1 Resistance, 2 Resistance2, 3 Ultrasonic
    uint8_t digital; // 0 analog value, 1 digital 0/1
};

struct PairCfg
{
    uint8_t motor_mode;   // 1 = M<n> is a motor, 0 = O<2n-1>/O<2n> are two single outputs
    uint8_t counter_edge; // counter C<n> counts on 0: rising edge (0 -> 1), 1: falling edge (1 -> 0)
};

struct TxConfig
{
    InputCfg i1, i2, i3, i4, i5, i6, i7, i8;
    PairCfg p1, p2, p3, p4;

    [[nodiscard]] const InputCfg& input(std::size_t i) const
    {
        const InputCfg* v[8] = {&i1, &i2, &i3, &i4, &i5, &i6, &i7, &i8};
        return *v[i];
    }
    [[nodiscard]] const PairCfg& pair(std::size_t p) const
    {
        const PairCfg* v[4] = {&p1, &p2, &p3, &p4};
        return *v[p];
    }
};

// Defaults match ftrobotx::Config: digital resistance inputs, motor mode, rising edge.
constexpr InputCfg kDefaultInput = {.mode = 1, .digital = 1};
constexpr PairCfg kDefaultPair = {.motor_mode = 1, .counter_edge = 0};
const TxConfig kDefaultConfig = {kDefaultInput, kDefaultInput, kDefaultInput, kDefaultInput,
                                 kDefaultInput, kDefaultInput, kDefaultInput, kDefaultInput,
                                 kDefaultPair,  kDefaultPair,  kDefaultPair,  kDefaultPair};

// ---------------------------------------------------------------------------------------------------------------
// Measurements: one instance per controller, in a global array (fixed addresses, absolute addressing), plus the
// link's values. Written by the loop, read by XCP DAQ in XcpEvent() on the same thread.

struct TxMeasurements
{
    uint16_t input[8];   // I1..I8 (meaning depends on <name>_config.iN)
    uint16_t counter[4]; // C1..C4 counter values
    uint16_t button_left;
    uint16_t button_right;
};

TxMeasurements gMeasurements[kMaxControllers] = {};
uint32_t link_cycle_count = 0;       // cycles since start
uint16_t link_config_generation = 0; // +1 per configuration change sent: marks mode switches in recordings
double link_cycles_per_s = 0;        // measured cycle rate (~100/s)
double link_round_trip_ms = 0;       // last relayed command (0 in online mode)
uint32_t link_resends = 0;           // relayed commands sent again
uint32_t link_lost_frames = 0;       // relayed value frames lost

tXcpEventId gCycleEvent = XCP_UNDEFINED_EVENT_ID; // one event per cycle

// ---------------------------------------------------------------------------------------------------------------

/// A controller with its calibration segments and what was last applied to it.
struct Bridged
{
    TxLink::Controller controller;
    TxMeasurements* measurements = nullptr;
    std::string outputsName; // segment names; XCPlite keeps pointers to them
    std::string configName;
    std::optional<xcp::CalSeg<TxOutputs>> outputs;
    std::optional<xcp::CalSeg<TxConfig>> config;
    std::optional<TxConfig> appliedConfig;
    std::optional<TxOutputs> appliedOutputs;
};

void registerTypes()
{
    // Enum conversions: CANape shows dropdowns instead of raw numbers.
    A2lCreateEnumConversion(InputModeConv, "4 0 \"Voltage\" 1 \"Resistance\" 2 \"Resistance2\" 3 \"Ultrasonic\"");
    A2lCreateEnumConversion(DigitalConv, "2 0 \"Analog\" 1 \"Digital\"");
    A2lCreateEnumConversion(PairModeConv, "2 0 \"Single outputs\" 1 \"Motor\"");
    A2lCreateEnumConversion(EdgeConv, "2 0 \"Rising (0->1)\" 1 \"Falling (1->0)\"");

    A2lCreateTypedef(TxOutputs, "ROBO TX output values", A2L_PARAMETER_COMPONENT(m1, "Motor M1 duty", "", -512, 512),
                     A2L_PARAMETER_COMPONENT(m2, "Motor M2 duty", "", -512, 512),
                     A2L_PARAMETER_COMPONENT(m3, "Motor M3 duty", "", -512, 512),
                     A2L_PARAMETER_COMPONENT(m4, "Motor M4 duty", "", -512, 512),
                     A2L_PARAMETER_COMPONENT(o1, "Output O1 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o2, "Output O2 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o3, "Output O3 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o4, "Output O4 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o5, "Output O5 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o6, "Output O6 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o7, "Output O7 duty", "", 0, 512),
                     A2L_PARAMETER_COMPONENT(o8, "Output O8 duty", "", 0, 512));

    // Leaf typedefs first, then the struct that nests them.
    A2lCreateTypedef(InputCfg, "Universal input configuration",
                     A2L_PARAMETER_COMPONENT(mode, "Input mode", "conv.InputModeConv", 0, 3),
                     A2L_PARAMETER_COMPONENT(digital, "Digital or analog value", "conv.DigitalConv", 0, 1));
    A2lCreateTypedef(PairCfg, "Motor/output pair and counter configuration",
                     A2L_PARAMETER_COMPONENT(motor_mode, "Motor or two single outputs", "conv.PairModeConv", 0, 1),
                     A2L_PARAMETER_COMPONENT(counter_edge, "Counter edge", "conv.EdgeConv", 0, 1));
    A2lCreateTypedef(TxConfig, "ROBO TX IO configuration", A2L_TYPEDEF_COMPONENT(i1, "InputCfg", 1),
                     A2L_TYPEDEF_COMPONENT(i2, "InputCfg", 1), A2L_TYPEDEF_COMPONENT(i3, "InputCfg", 1),
                     A2L_TYPEDEF_COMPONENT(i4, "InputCfg", 1), A2L_TYPEDEF_COMPONENT(i5, "InputCfg", 1),
                     A2L_TYPEDEF_COMPONENT(i6, "InputCfg", 1), A2L_TYPEDEF_COMPONENT(i7, "InputCfg", 1),
                     A2L_TYPEDEF_COMPONENT(i8, "InputCfg", 1), A2L_TYPEDEF_COMPONENT(p1, "PairCfg", 1),
                     A2L_TYPEDEF_COMPONENT(p2, "PairCfg", 1), A2L_TYPEDEF_COMPONENT(p3, "PairCfg", 1),
                     A2L_TYPEDEF_COMPONENT(p4, "PairCfg", 1));

    A2lCreateTypedef(TxMeasurements, "ROBO TX inputs",
                     A2L_MEASUREMENT_ARRAY_COMPONENT(input, "Universal inputs I1..I8", ""),
                     A2L_MEASUREMENT_ARRAY_COMPONENT(counter, "Counter values C1..C4", ""),
                     A2L_MEASUREMENT_COMPONENT(button_left, "Left display button (time pressed, 10 ms)", ""),
                     A2L_MEASUREMENT_COMPONENT(button_right, "Right display button (time pressed, 10 ms)", ""));
}

void registerSegments(Bridged& bridged)
{
    std::string comment = "ROBO TX outputs of " + bridged.controller.description;
    bridged.outputs.emplace(bridged.outputsName.c_str(), &kDefaultOutputs);
    bridged.outputs->CreateA2lTypedefInstance("TxOutputs", comment.c_str());
    comment = "ROBO TX IO configuration of " + bridged.controller.description;
    bridged.config.emplace(bridged.configName.c_str(), &kDefaultConfig);
    bridged.config->CreateA2lTypedefInstance("TxConfig", comment.c_str());
}

// All measurements in one block: each addressing mode block makes its own A2L group, and the group names would
// repeat.
void registerMeasurements(std::vector<Bridged>& bridged)
{
    A2lLock();
    A2lSetAbsoluteAddrMode_i(gCycleEvent);
    for (Bridged& b : bridged)
    {
        std::string comment = "Inputs of " + b.controller.description;
        A2lCreateInstance_(b.controller.name.c_str(), "TxMeasurements", 0, b.measurements, comment.c_str());
    }
    A2lCreateMeasurement(link_cycle_count, "Cycles since start");
    A2lCreateMeasurement(link_config_generation, "Incremented whenever a new IO configuration was sent");
    A2lCreatePhysMeasurement(link_cycles_per_s, "Cycle rate", "1/s", 0.0, 200.0);
    A2lCreatePhysMeasurement(link_round_trip_ms, "Round trip of the last relayed command", "ms", 0.0, 1000.0);
    A2lCreateMeasurement(link_resends, "Relayed commands sent again");
    A2lCreateMeasurement(link_lost_frames, "Relayed value frames lost");
    A2lUnlock();
}

// Applies the configuration segment if it changed; returns which motor pairs switched mode, so their outputs get
// applied again.
std::array<bool, kMotors> applyConfig(Bridged& bridged)
{
    std::array<bool, kMotors> pairSwitched{};
    TxConfig now;
    {
        auto config = bridged.config->lock(); // keep the lock short, XCP writes wait for it
        now = *config;
    }

    TxIo& io = *bridged.controller.io;
    const std::optional<TxConfig>& applied = bridged.appliedConfig;
    bool changed = false;
    for (std::size_t i = 0; i < kUniversalInputs; ++i)
    {
        const InputCfg& in = now.input(i);
        if (applied && in.mode == applied->input(i).mode && in.digital == applied->input(i).digital)
        {
            continue;
        }
        uint8_t mode = in.mode <= static_cast<uint8_t>(InputMode::Ultrasonic) ? in.mode : uint8_t{1};
        io.setInputMode(ExtensionIndex::Local, static_cast<UniversalInput>(i), static_cast<InputMode>(mode),
                        in.digital != 0);
        changed = true;
    }
    for (std::size_t p = 0; p < kMotors; ++p)
    {
        const PairCfg& pair = now.pair(p);
        if (!applied || pair.motor_mode != applied->pair(p).motor_mode)
        {
            io.setMotorMode(ExtensionIndex::Local, static_cast<Motor>(p), pair.motor_mode != 0); // stops the pair
            pairSwitched[p] = true;
            changed = true;
        }
        if (!applied || pair.counter_edge != applied->pair(p).counter_edge)
        {
            io.setCounterInverted(ExtensionIndex::Local, static_cast<Counter>(p), pair.counter_edge != 0);
            changed = true;
        }
    }

    if (changed)
    {
        ++link_config_generation;
    }
    bridged.appliedConfig = now;
    return pairSwitched;
}

// Applies the output segment; only sends values that changed or whose pair just switched mode.
void applyOutputs(Bridged& bridged, const std::array<bool, kMotors>& pairSwitched)
{
    TxOutputs now;
    {
        auto outputs = bridged.outputs->lock();
        now = *outputs;
    }

    TxIo& io = *bridged.controller.io;
    const std::optional<TxOutputs>& applied = bridged.appliedOutputs;
    for (std::size_t p = 0; p < kMotors; ++p)
    {
        const bool force = !applied || pairSwitched[p];
        if (bridged.appliedConfig->pair(p).motor_mode != 0)
        {
            if (force || now.motor(p) != applied->motor(p))
            {
                io.setMotorOutput(ExtensionIndex::Local, static_cast<Motor>(p), now.motor(p));
            }
        }
        else
        {
            for (std::size_t o = 2 * p; o < 2 * p + 2; ++o)
            {
                if (force || now.output(o) != applied->output(o))
                {
                    io.setPwmOutput(ExtensionIndex::Local, static_cast<PWM>(o), now.output(o));
                }
            }
        }
    }
    bridged.appliedOutputs = now;
}

// Copies a controller's inputs into its measurements.
void mirrorInputs(Bridged& bridged)
{
    const TxIo& io = *bridged.controller.io;
    TxMeasurements& m = *bridged.measurements;
    for (std::size_t i = 0; i < kUniversalInputs; ++i)
    {
        m.input[i] = io.analogValue(ExtensionIndex::Local, static_cast<UniversalInput>(i));
    }
    for (std::size_t c = 0; c < kMotors; ++c)
    {
        m.counter[c] = io.counterValue(ExtensionIndex::Local, static_cast<Counter>(c));
    }
    m.button_left = io.displayButtonLeft(ExtensionIndex::Local);
    m.button_right = io.displayButtonRight(ExtensionIndex::Local);
}

int run(const std::string& portOption, const std::string& addressOption)
{
    std::string portName = cli::choosePort(portOption);
    if (portName.empty())
    {
        return 1;
    }

    std::unique_ptr<TxLink> link;
    if (addressOption.empty())
    {
        link = std::make_unique<OnlineLink>(portName);
    }
    else
    {
        std::optional<BtAddress> address = parseBtAddress(addressOption);
        if (!address)
        {
            std::cerr << "Invalid Bluetooth address " << addressOption << '\n';
            return 2;
        }
        std::cout << "Connecting to the model TX " << toString(*address) << " through the relay on " << portName
                  << "...\n";
        link = std::make_unique<RelayLink>(portName, *address);
    }

    // The A2L description: types, one set of segments and measurements per controller, the link's values.
    gCycleEvent = XcpCreateEvent("tx_cycle", 0 /*sporadic, about 100/s*/, 0);
    registerTypes();
    std::vector<TxLink::Controller> controllers = link->controllers();
    std::vector<Bridged> bridged(std::min(controllers.size(), kMaxControllers));
    for (std::size_t i = 0; i < bridged.size(); ++i)
    {
        bridged[i].controller = controllers[i];
        bridged[i].measurements = &gMeasurements[i];
        bridged[i].outputsName = controllers[i].name + "_outputs";
        bridged[i].configName = controllers[i].name + "_config";
        registerSegments(bridged[i]);
        std::cout << controllers[i].name << ": " << controllers[i].description << '\n';
    }
    registerMeasurements(bridged);
    std::cout << "XCP server on UDP " << kServerPort << ". Press Ctrl+C to quit.\n";

    auto rateStart = std::chrono::steady_clock::now();
    uint32_t rateCycles = 0;
    auto nextReconnect = std::chrono::steady_clock::now();
    bool wasConnected = true;
    try
    {
        while (stopRequested == 0)
        {
            // A lost model connection doesn't end the session: the relay's signals keep coming, and after reconnecting
            // CANape's configuration and outputs are sent again.
            if (!link->connected())
            {
                if (wasConnected)
                {
                    std::cerr << "Connection to the model TX lost; trying to reconnect\n";
                    wasConnected = false;
                }
                if (std::chrono::steady_clock::now() >= nextReconnect)
                {
                    if (link->reconnect())
                    {
                        std::cerr << "Reconnected\n";
                        wasConnected = true;
                        for (Bridged& b : bridged)
                        {
                            b.appliedConfig.reset();
                            b.appliedOutputs.reset();
                        }
                    }
                    nextReconnect = std::chrono::steady_clock::now() + std::chrono::seconds(3);
                }
            }

            for (Bridged& b : bridged)
            {
                auto pairSwitched = applyConfig(b); // CANape config -> configuration
                applyOutputs(b, pairSwitched);      // CANape outputs -> outputs
            }
            link->update(); // one cycle: send configuration/outputs, read inputs
            for (Bridged& b : bridged)
            {
                mirrorInputs(b);
            }

            TxLink::Stats stats = link->stats();
            link_round_trip_ms = stats.roundTripMs;
            link_resends = stats.resends;
            link_lost_frames = stats.lostFrames;
            ++link_cycle_count;
            ++rateCycles;
            auto now = std::chrono::steady_clock::now();
            if (now - rateStart >= std::chrono::seconds(1))
            {
                link_cycles_per_s = rateCycles / std::chrono::duration<double>(now - rateStart).count();
                rateStart = now;
                rateCycles = 0;
            }

            // Timestamped with the PC clock now. With the VM on the model, its own tick per sample would be passed
            // instead: XcpEventAt(gCycleEvent, tickToClock(sample.tick)).
            XcpEvent(gCycleEvent);
        }
    }
    catch (...)
    {
        link->stopAllOutputs();
        throw;
    }
    link->stopAllOutputs();
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    std::string portOption;
    std::string addressOption;
    int exitCode = 0;
    if (!cli::parse(argc, argv, "[-p PORT] [-a MODEL_ADDRESS]", DESCRIPTION,
                    {{"-p", "--port", "PORT", cli::kPortHelp, &portOption},
                     {"-a", "--address", "ADDRESS",
                      "Bluetooth address of a model TX running RelayModel; PORT is then the relay TX.\n"
                      "                        Example: -p /dev/ttyACM0 -a 00:13:7B:48:B2:E2.",
                      &addressOption}},
                    nullptr, exitCode))
    {
        return exitCode;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // XCP first, so the A2L generator can see all registrations.
    XcpSetLogLevel(2);
    if (!XcpInit("ftrobotx", "V1.2", XCP_MODE_LOCAL) ||
        !XcpEthServerInit(kServerAddr, kServerPort, /*use_tcp=*/false, kQueueSize) ||
        !A2lInit(kServerAddr, kServerPort, false,
                 A2L_MODE_WRITE_ALWAYS | A2L_MODE_FINALIZE_ON_CONNECT | A2L_MODE_AUTO_GROUPS))
    {
        std::cerr << "Error: could not start the XCP server\n";
        return 1;
    }

    try
    {
        exitCode = run(portOption, addressOption);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        exitCode = 1;
    }

    // On every way out, so the A2L file is complete.
    XcpDisconnect();
    A2lFinalize();
    XcpEthServerShutdown();
    return exitCode;
}
