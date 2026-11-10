// How the PC reaches ROBO TX Controllers, independent of the mode:
//
//   - TxIo: the inputs and outputs of one controller.
//   - TxLink: a connection that runs the cycle and provides one or more named controllers:
//       OnlineLink  "tx"               one controller in online mode (USB or Bluetooth)
//       RelayLink   "relay", "model"   the relay TX in online mode, and the model TX behind it (RelayModel)
//     Later also a direct link to a model TX, without the relay.
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ftrobotx/FtConnection.hpp"
#include "ftrobotx/relay/RelayConnection.hpp"

namespace ftrobotx
{

/// The inputs and outputs of one controller (and its extensions, by ExtensionIndex).
class TxIo
{
  public:
    virtual ~TxIo() = default;

    virtual void setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital) = 0;
    /// A pair of outputs as motor (true) or as two single outputs (false); switching stops it.
    virtual void setMotorMode(ExtensionIndex index, Motor motor, bool motorMode) = 0;
    virtual void setCounterInverted(ExtensionIndex index, Counter counter, bool inverted) = 0;
    virtual void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty) = 0;
    virtual void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty) = 0;

    [[nodiscard]] virtual std::uint16_t analogValue(ExtensionIndex index, UniversalInput input) const = 0;
    [[nodiscard]] virtual std::uint16_t counterValue(ExtensionIndex index, Counter counter) const = 0;
    [[nodiscard]] virtual std::uint16_t displayButtonLeft(ExtensionIndex index) const = 0;
    [[nodiscard]] virtual std::uint16_t displayButtonRight(ExtensionIndex index) const = 0;
};

/// A connection: runs the cycle for all its controllers.
class TxLink
{
  public:
    struct Controller
    {
        std::string name; // e.g. "tx", "relay", "model": prefix in the A2L
        std::string description;
        TxIo* io;
    };

    /// State of the link, for display and measurement.
    struct Stats
    {
        double roundTripMs = 0;       // last relayed command; 0 in online mode
        std::uint32_t resends = 0;    // relayed commands sent again
        std::uint32_t lostFrames = 0; // relayed value frames lost
    };

    virtual ~TxLink() = default;

    [[nodiscard]] virtual std::vector<Controller> controllers() = 0;
    [[nodiscard]] virtual bool connected() const = 0;
    /// One cycle for all controllers: sends what changed, reads the inputs. Takes about 10 ms (the TX refreshes its
    /// I/O that often; a relay must not be polled faster).
    virtual void update() = 0;
    virtual void stopAllOutputs() = 0;
    /// Tries to connect again after connected() went false; true on success. Blocks a few seconds.
    virtual bool reconnect()
    {
        return false;
    }
    [[nodiscard]] virtual Stats stats() const
    {
        return {};
    }
};

/// TxIo of a controller in online mode; the FtConnection's owner runs its cycle.
class OnlineTxIo : public TxIo
{
  public:
    explicit OnlineTxIo(FtConnection& ft) : ft_(ft)
    {
    }

    void setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital) override
    {
        ft_.setInputMode(index, input, mode, digital);
    }
    void setMotorMode(ExtensionIndex index, Motor motor, bool motorMode) override
    {
        Extension& ext = ft_.extension(index);
        auto m = static_cast<std::size_t>(motor);
        ext.setMotorOutput(m, 0); // ends a distance/sync command; a partner motor stops
        ext.output.pwmOutputValues.at(2 * m) = 0;
        ext.output.pwmOutputValues.at(2 * m + 1) = 0;
        ext.config.setMotorConfig(m, motorMode ? 1 : 0);
    }
    void setCounterInverted(ExtensionIndex index, Counter counter, bool inverted) override
    {
        ft_.extension(index).config.setCounterConfig(static_cast<std::size_t>(counter), inverted ? 1 : 0);
    }
    void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty) override
    {
        ft_.setMotorOutput(index, motor, duty);
    }
    void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty) override
    {
        ft_.setPwmOutput(index, pwm, duty);
    }

    [[nodiscard]] std::uint16_t analogValue(ExtensionIndex index, UniversalInput input) const override
    {
        return ft_.extension(index).input.universalInput(static_cast<std::size_t>(input));
    }
    [[nodiscard]] std::uint16_t counterValue(ExtensionIndex index, Counter counter) const override
    {
        return ft_.counterValue(index, counter);
    }
    [[nodiscard]] std::uint16_t displayButtonLeft(ExtensionIndex index) const override
    {
        return ft_.extension(index).input.displayButtonLeft();
    }
    [[nodiscard]] std::uint16_t displayButtonRight(ExtensionIndex index) const override
    {
        return ft_.extension(index).input.displayButtonRight();
    }

  private:
    FtConnection& ft_;
};

/// TxIo of a model TX behind a relay; the RelayConnection's owner runs its cycle.
class RelayModelTxIo : public TxIo
{
  public:
    explicit RelayModelTxIo(RelayConnection& model) : model_(model)
    {
    }

    void setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital) override
    {
        model_.setInputMode(index, input, mode, digital);
    }
    void setMotorMode(ExtensionIndex index, Motor motor, bool motorMode) override
    {
        model_.setMotorMode(index, motor, motorMode);
    }
    void setCounterInverted(ExtensionIndex index, Counter counter, bool inverted) override
    {
        model_.setCounterInverted(index, counter, inverted);
    }
    void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty) override
    {
        model_.setMotorOutput(index, motor, duty);
    }
    void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty) override
    {
        model_.setPwmOutput(index, pwm, duty);
    }

    [[nodiscard]] std::uint16_t analogValue(ExtensionIndex index, UniversalInput input) const override
    {
        return model_.analogValue(index, input);
    }
    [[nodiscard]] std::uint16_t counterValue(ExtensionIndex index, Counter counter) const override
    {
        return model_.counterValue(index, counter);
    }
    [[nodiscard]] std::uint16_t displayButtonLeft(ExtensionIndex index) const override
    {
        return model_.value(index, RELAY_SIG_BUTTON_LEFT);
    }
    [[nodiscard]] std::uint16_t displayButtonRight(ExtensionIndex index) const override
    {
        return model_.value(index, RELAY_SIG_BUTTON_RIGHT);
    }

  private:
    RelayConnection& model_;
};

/// One controller in online mode.
class OnlineLink : public TxLink
{
  public:
    explicit OnlineLink(const std::string& port) : port_(port), ft_(port), io_(ft_)
    {
    }

    [[nodiscard]] std::vector<Controller> controllers() override
    {
        return {{"tx", ft_.extensions().front().name + " (online, " + port_ + ")", &io_}};
    }
    [[nodiscard]] bool connected() const override
    {
        return true; // FtConnection throws when the controller stops answering
    }
    void update() override
    {
        std::this_thread::sleep_until(nextCycle_);
        nextCycle_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
        ft_.update();
    }
    void stopAllOutputs() override
    {
        ft_.stopAllOutputs();
        ft_.update();
    }

  private:
    std::string port_;
    FtConnection ft_;
    OnlineTxIo io_;
    std::chrono::steady_clock::time_point nextCycle_;
};

/// The relay TX in online mode and the model TX behind it (running RelayModel). Both have inputs and outputs: the
/// relay could be a remote control (steering wheel, pedals) for the model.
class RelayLink : public TxLink
{
  public:
    RelayLink(const RelayLink&) = delete;
    RelayLink& operator=(const RelayLink&) = delete;
    RelayLink(const std::string& relayPort, const BtAddress& modelAddress)
        : port_(relayPort), modelAddress_(modelAddress), relay_(relayPort), model_(relay_, modelAddress),
          relayIo_(relay_), modelIo_(model_)
    {
        model_.connect();
        model_.subscribe(ExtensionIndex::Local, kRelayAll);
    }

    [[nodiscard]] std::vector<Controller> controllers() override
    {
        return {{"relay", relay_.extensions().front().name + " (relay, online, " + port_ + ")", &relayIo_},
                {"model", "model TX " + toString(modelAddress_) + " (through the relay)", &modelIo_}};
    }
    [[nodiscard]] bool connected() const override
    {
        return model_.connected();
    }
    void update() override
    {
        model_.update(); // runs the relay's online cycle too, at most every 10 ms
    }
    ~RelayLink() override
    {
        // Close the Bluetooth connection, so the model listens again right away for the next session.
        try
        {
            model_.disconnect();
            relay_.update();
        }
        catch (...)
        {
        }
    }

    void stopAllOutputs() override
    {
        relay_.stopAllOutputs();
        if (model_.connected())
        {
            model_.stopAllOutputs();
            model_.flush();
        }
    }
    bool reconnect() override
    {
        try
        {
            model_.connect();
            model_.subscribe(ExtensionIndex::Local, kRelayAll);
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }
    [[nodiscard]] Stats stats() const override
    {
        const RelayConnection::Statistics& stats = model_.statistics();
        return {.roundTripMs = stats.lastRoundTripMs, .resends = stats.resends, .lostFrames = stats.lostValueFrames};
    }

  private:
    std::string port_;
    BtAddress modelAddress_;
    FtConnection relay_;
    RelayConnection model_;
    OnlineTxIo relayIo_;
    RelayModelTxIo modelIo_;
};

} // namespace ftrobotx
