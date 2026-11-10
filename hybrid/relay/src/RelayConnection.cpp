#include "ftrobotx/relay/RelayConnection.hpp"

#include <algorithm>
#include <stdexcept>
#include <thread>
#include <utility>

#include "ftrobotx/Sensors.hpp"

namespace ftrobotx
{

namespace
{

void putU16(std::vector<std::uint8_t>& frame, std::uint16_t value)
{
    frame.push_back(static_cast<std::uint8_t>(value));
    frame.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::uint8_t ctrlOf(ExtensionIndex index)
{
    return static_cast<std::uint8_t>(index);
}

/// Key of a setting: op, controller, index. Settings with the same key replace each other.
std::uint32_t keyOf(std::uint8_t op, std::uint8_t ctrl, std::uint8_t index)
{
    return static_cast<std::uint32_t>(op) << 16 | static_cast<std::uint32_t>(ctrl) << 8 | index;
}

} // namespace

RelayConnection::RelayConnection(FtConnection& relay, const BtAddress& model, Options options)
    : relay_(relay), model_(model), options_(options)
{
    relay_.setBtCallback([this](const BtEvent& event) {
        if (event.channel != options_.channel)
        {
            return;
        }
        switch (event.status)
        {
        case BtStatus::MessageIndication:
            received_.push_back(event.message);
            break;
        case BtStatus::DisconnectIndication:
            connected_ = false;
            break;
        default:
            if (event.command == BtCommand::Connect)
            {
                connectResult_ = event.status;
            }
            else if (event.command == BtCommand::Send && event.status == BtStatus::ConnectionInvalid)
            {
                connected_ = false; // the connection died without a report
            }
            break;
        }
    });
}

void RelayConnection::connect(std::chrono::milliseconds timeout)
{
    auto deadline = Clock::now() + timeout;
    auto waitFor = [&](const std::function<bool()>& done) {
        while (!done() && Clock::now() < deadline)
        {
            std::this_thread::sleep_until(nextRelayCycle_);
            nextRelayCycle_ = Clock::now() + options_.relayPeriod;
            relay_.update();
            if (!received_.empty())
            {
                for (const auto& frame : std::exchange(received_, {}))
                {
                    handleFrame(frame);
                }
            }
        }
        return done();
    };

    if (std::optional<BtAddress> own = parseBtAddress(relay_.extensions().front().macAddress); own && *own == model_)
    {
        throw std::runtime_error(toString(model_) + " is the relay itself; give the address of the model TX");
    }

    // A connection left over from an earlier run would make the model ignore our first sequence numbers.
    relay_.btDisconnect(options_.channel);
    connectResult_.reset();
    relay_.btConnect(options_.channel, model_);
    if (!waitFor([&] { return connectResult_.has_value(); }) ||
        (*connectResult_ != BtStatus::Success && *connectResult_ != BtStatus::ConnectionExists))
    {
        throw std::runtime_error(std::string("No Bluetooth connection to the model TX ") + toString(model_) + ": " +
                                 (connectResult_ ? toString(*connectResult_) : "timeout") +
                                 " (is relay_model running there, built for this relay's address?)");
    }
    relay_.btStartReceive(options_.channel);
    connected_ = true;

    queue_.clear();
    inFlight_.reset();
    inEffect_.clear();
    helloReply_.reset();
    echoed_ = false;
    lastValuesSeq_.reset(); // a restarted model counts from 0 again
    hasValues_.fill(false);
    queueCommand({RELAY_HELLO, RELAY_PROTOCOL_VERSION});
    if (!waitFor([&] {
            sendNext();
            return helloReply_.has_value() || echoed_ || idle(); // idle: HELLO given up
        }) ||
        !helloReply_)
    {
        connected_ = false;
        relay_.btDisconnect(options_.channel);
        throw std::runtime_error(echoed_ ? "The model TX sends our commands back unchanged: it runs BtEcho, not "
                                           "RelayModel (hybrid/relay/tx)"
                                         : "The model TX doesn't answer: is RelayModel (hybrid/relay/tx) running "
                                           "there, built for this relay's address?");
    }
    const std::vector<std::uint8_t>& hello = *helloReply_;
    if (hello.size() < 6 || hello[3] != RELAY_PROTOCOL_VERSION)
    {
        throw std::runtime_error("The model TX speaks another protocol version");
    }
    controllers_ = hello[4];
    for (std::size_t i = 0; i < extensionsOnline_.size(); ++i)
    {
        extensionsOnline_[i] = (hello[5] >> i & 1) != 0;
    }
}

void RelayConnection::disconnect()
{
    relay_.btDisconnect(options_.channel);
    connected_ = false;
    queue_.clear();
    inFlight_.reset();
}

void RelayConnection::queueSetting(std::uint32_t key, std::vector<std::uint8_t> frame)
{
    auto queued = std::ranges::find_if(queue_, [&](const Command& command) { return command.key == key; });
    if (queued != queue_.end())
    {
        queued->frame = std::move(frame);
        return;
    }
    // Nothing to do if the model has it already, or it is on its way.
    const std::vector<std::uint8_t>* current = nullptr;
    if (inFlight_ && inFlight_->key == key)
    {
        current = &inFlight_->frame;
    }
    else if (auto known = inEffect_.find(key); known != inEffect_.end())
    {
        current = &known->second;
    }
    if (current != nullptr && *current == frame)
    {
        return;
    }
    queue_.push_back(Command{.frame = std::move(frame), .key = key});
}

void RelayConnection::queueCommand(std::vector<std::uint8_t> frame)
{
    queue_.push_back(Command{.frame = std::move(frame)});
}

void RelayConnection::forgetMotorSettings(int ctrl)
{
    std::erase_if(inEffect_, [&](const auto& entry) {
        auto op = static_cast<std::uint8_t>(entry.first >> 16);
        bool motor = op == RELAY_SET_MOTOR || op == RELAY_SET_PWM || op == RELAY_MOTOR_MODE;
        return motor && (ctrl < 0 || static_cast<int>(entry.first >> 8 & 0xFF) == ctrl);
    });
}

void RelayConnection::setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty)
{
    std::uint8_t ctrl = ctrlOf(index);
    auto output = static_cast<std::uint8_t>(pwm);
    std::vector<std::uint8_t> frame{RELAY_SET_PWM, ctrl, output};
    putU16(frame, static_cast<std::uint16_t>(duty));
    queueSetting(keyOf(RELAY_SET_PWM, ctrl, output), std::move(frame));
}

void RelayConnection::setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty)
{
    std::uint8_t ctrl = ctrlOf(index);
    auto m = static_cast<std::uint8_t>(motor);
    std::vector<std::uint8_t> frame{RELAY_SET_MOTOR, ctrl, m};
    putU16(frame, static_cast<std::uint16_t>(duty));
    queueSetting(keyOf(RELAY_SET_MOTOR, ctrl, m), std::move(frame));
}

void RelayConnection::startMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance)
{
    std::vector<std::uint8_t> frame{RELAY_MOTOR_DISTANCE, ctrlOf(index), static_cast<std::uint8_t>(motor)};
    putU16(frame, static_cast<std::uint16_t>(duty));
    putU16(frame, distance);
    forgetMotorSettings(ctrlOf(index));
    queueCommand(std::move(frame));
}

void RelayConnection::startMotorSync(ExtensionIndex index, Motor master, Motor slave, std::int16_t duty,
                                     std::uint16_t distance, SyncDirection direction)
{
    if (master == slave)
    {
        throw std::invalid_argument("A motor can't be synchronized with itself");
    }
    std::vector<std::uint8_t> frame{RELAY_MOTOR_SYNC, ctrlOf(index), static_cast<std::uint8_t>(master),
                                    static_cast<std::uint8_t>(slave)};
    putU16(frame, static_cast<std::uint16_t>(duty));
    putU16(frame, distance);
    frame.push_back(direction == SyncDirection::Opposite ? 1 : 0);
    forgetMotorSettings(ctrlOf(index));
    queueCommand(std::move(frame));
}

void RelayConnection::stopMotor(ExtensionIndex index, Motor motor)
{
    forgetMotorSettings(ctrlOf(index));
    queueCommand({RELAY_STOP_MOTOR, ctrlOf(index), static_cast<std::uint8_t>(motor)});
}

void RelayConnection::stopAllOutputs()
{
    // Anything still queued for the outputs would undo the stop.
    std::erase_if(queue_, [](const Command& command) {
        std::uint8_t op = command.frame[0];
        return op == RELAY_SET_PWM || op == RELAY_SET_MOTOR || op == RELAY_MOTOR_DISTANCE || op == RELAY_MOTOR_SYNC;
    });
    forgetMotorSettings(-1);
    queueCommand({RELAY_STOP_ALL});
}

void RelayConnection::resetCounter(ExtensionIndex index, Counter counter)
{
    queueCommand({RELAY_RESET_COUNTER, ctrlOf(index), static_cast<std::uint8_t>(counter)});
}

void RelayConnection::setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital)
{
    std::uint8_t ctrl = ctrlOf(index);
    auto i = static_cast<std::uint8_t>(input);
    queueSetting(keyOf(RELAY_INPUT_MODE, ctrl, i),
                 {RELAY_INPUT_MODE, ctrl, i, static_cast<std::uint8_t>(mode), static_cast<std::uint8_t>(digital)});
}

void RelayConnection::setMotorMode(ExtensionIndex index, Motor motor, bool motor_mode)
{
    std::uint8_t ctrl = ctrlOf(index);
    auto m = static_cast<std::uint8_t>(motor);
    forgetMotorSettings(ctrl);
    queueSetting(keyOf(RELAY_MOTOR_MODE, ctrl, m), {RELAY_MOTOR_MODE, ctrl, m, static_cast<std::uint8_t>(motor_mode)});
}

void RelayConnection::setCounterInverted(ExtensionIndex index, Counter counter, bool inverted)
{
    std::uint8_t ctrl = ctrlOf(index);
    auto c = static_cast<std::uint8_t>(counter);
    queueSetting(keyOf(RELAY_COUNTER_MODE, ctrl, c),
                 {RELAY_COUNTER_MODE, ctrl, c, static_cast<std::uint8_t>(inverted)});
}

void RelayConnection::showMessage(std::string_view text)
{
    constexpr std::size_t kPiece = RELAY_FRAME_MAX - 4; // op, seq, offset, last
    text = text.substr(0, 98);
    std::size_t offset = 0;
    do
    {
        std::string_view piece = text.substr(offset, kPiece);
        bool last = offset + piece.size() >= text.size();
        std::vector<std::uint8_t> frame{RELAY_TEXT, static_cast<std::uint8_t>(offset), static_cast<std::uint8_t>(last)};
        frame.insert(frame.end(), piece.begin(), piece.end());
        queueCommand(std::move(frame));
        offset += piece.size();
    } while (offset < text.size());
}

void RelayConnection::subscribe(ExtensionIndex index, std::uint32_t signals, std::chrono::milliseconds interval)
{
    std::uint8_t ctrl = ctrlOf(index);
    std::vector<std::uint8_t> frame{RELAY_SUBSCRIBE, ctrl};
    for (int shift = 0; shift < 32; shift += 8)
    {
        frame.push_back(static_cast<std::uint8_t>(signals >> shift));
    }
    putU16(frame, static_cast<std::uint16_t>(std::clamp<long long>(interval.count(), 0, 65535)));
    hasValues_.at(ctrl) = false;
    queueSetting(keyOf(RELAY_SUBSCRIBE, ctrl, 0), std::move(frame));
}

std::uint16_t RelayConnection::value(ExtensionIndex index, RelaySignal signal) const
{
    return values_.at(static_cast<std::size_t>(index)).at(static_cast<std::size_t>(signal));
}

std::uint16_t RelayConnection::analogValue(ExtensionIndex index, UniversalInput input) const
{
    return value(index, static_cast<RelaySignal>(RELAY_SIG_I1 + static_cast<int>(input)));
}

bool RelayConnection::digitalValue(ExtensionIndex index, UniversalInput input) const
{
    return analogValue(index, input) != 0;
}

std::uint16_t RelayConnection::counterValue(ExtensionIndex index, Counter counter) const
{
    return value(index, static_cast<RelaySignal>(RELAY_SIG_C1 + static_cast<int>(counter)));
}

bool RelayConnection::motorCommandDone(ExtensionIndex index, Motor motor) const
{
    return value(index, static_cast<RelaySignal>(RELAY_SIG_M1_DONE + static_cast<int>(motor))) != 0;
}

std::optional<double> RelayConnection::ntcTemperature(ExtensionIndex index, UniversalInput input, double r25,
                                                      double b) const
{
    return ntcCelsius(analogValue(index, input), r25, b);
}

std::optional<std::uint16_t> RelayConnection::ultrasonicDistance(ExtensionIndex index, UniversalInput input) const
{
    return ultrasonicCm(analogValue(index, input));
}

bool RelayConnection::hasValues(ExtensionIndex index) const
{
    return hasValues_.at(static_cast<std::size_t>(index));
}

void RelayConnection::update()
{
    std::this_thread::sleep_until(nextRelayCycle_);
    nextRelayCycle_ = Clock::now() + options_.relayPeriod;
    relay_.update();
    for (const auto& frame : std::exchange(received_, {}))
    {
        handleFrame(frame);
    }
    if (!connected_)
    {
        return;
    }
    if (idle() && Clock::now() - lastActivity_ >= options_.pingInterval)
    {
        queueCommand({RELAY_PING});
    }
    sendNext();
}

bool RelayConnection::flush(std::chrono::milliseconds timeout)
{
    auto deadline = Clock::now() + timeout;
    while (!idle() && connected_ && Clock::now() < deadline)
    {
        update();
    }
    return idle();
}

void RelayConnection::sendNext()
{
    if (!connected_)
    {
        return;
    }
    if (inFlight_)
    {
        if (Clock::now() - inFlight_->lastSent < options_.ackTimeout)
        {
            return;
        }
        if (inFlight_->attempts >= options_.attempts)
        {
            std::uint8_t op = inFlight_->frame[0];
            inFlight_.reset();
            fail("No answer from the model TX to command 0x" + std::to_string(op));
            return;
        }
        ++statistics_.resends;
        transmit(*inFlight_);
        return;
    }
    if (queue_.empty())
    {
        return;
    }
    inFlight_ = std::move(queue_.front());
    queue_.pop_front();
    inFlight_->seq = nextSeq_;
    nextSeq_ = static_cast<std::uint8_t>(nextSeq_ == 255 ? 1 : nextSeq_ + 1);
    inFlight_->firstSent = Clock::now();
    transmit(*inFlight_);
}

void RelayConnection::transmit(Command& command)
{
    std::vector<std::uint8_t> frame = command.frame;
    frame.insert(frame.begin() + 1, command.seq);
    relay_.btSend(options_.channel, frame);
    command.lastSent = Clock::now();
    ++command.attempts;
    lastActivity_ = command.lastSent;
}

void RelayConnection::handleFrame(const std::vector<std::uint8_t>& frame)
{
    if (frame.size() < 2)
    {
        return;
    }
    if (frame[0] == RELAY_ACK && frame.size() >= 3)
    {
        if (!inFlight_ || frame[1] != inFlight_->seq)
        {
            return; // ACK of a resent command that was answered already
        }
        Command done = std::move(*inFlight_);
        inFlight_.reset();
        statistics_.lastRoundTripMs = std::chrono::duration<double, std::milli>(Clock::now() - done.firstSent).count();
        if (frame[2] != RELAY_OK)
        {
            fail("The model TX refused command 0x" + std::to_string(done.frame[0]) + " (status " +
                 std::to_string(frame[2]) + ")");
            return;
        }
        ++statistics_.commands;
        if (done.key)
        {
            inEffect_[*done.key] = done.frame;
        }
        if (done.frame[0] == RELAY_HELLO)
        {
            helloReply_ = frame;
        }
        sendNext();
        return;
    }
    if (frame[0] < RELAY_ACK)
    {
        echoed_ = true; // a command came back: the other side echoes instead of answering
        return;
    }
    if (frame[0] == RELAY_VALUES && frame.size() >= 5)
    {
        std::uint8_t seq = frame[1];
        if (lastValuesSeq_)
        {
            statistics_.lostValueFrames += static_cast<std::uint8_t>(seq - *lastValuesSeq_ - 1);
        }
        lastValuesSeq_ = seq;
        ++statistics_.valueFrames;

        std::size_t ctrl = frame[2];
        std::size_t first = frame[3];
        std::size_t count = frame[4];
        if (ctrl >= values_.size() || first + count > RELAY_SIG_COUNT || frame.size() < 5 + 2 * count)
        {
            return;
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            values_[ctrl][first + i] = static_cast<std::uint16_t>(frame[5 + 2 * i] | frame[6 + 2 * i] << 8);
        }
        hasValues_[ctrl] = true;
    }
}

void RelayConnection::fail(const std::string& message)
{
    ++statistics_.failures;
    if (errorCallback_)
    {
        errorCallback_(message);
    }
}

} // namespace ftrobotx
