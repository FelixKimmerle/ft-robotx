#include "ftrobotx/Extension.hpp"

#include <stdexcept>

namespace ftrobotx
{

void Extension::setMotorOutput(std::size_t motor, std::int16_t duty)
{
    releaseMotor(motor);
    output.pwmOutputValues.at(2 * motor) = duty > 0 ? duty : std::int16_t{0};
    output.pwmOutputValues.at(2 * motor + 1) = duty > 0 ? std::int16_t{0} : static_cast<std::int16_t>(-duty);
}

void Extension::startMotorDistance(std::size_t motor, std::int16_t duty, std::uint16_t distance)
{
    setMotorOutput(motor, duty);
    output.motorDistanceValues.at(motor) = distance;
    ++output.motorCommandId.at(motor);
}

void Extension::startMotorSync(std::size_t master, std::size_t slave, std::int16_t duty, std::uint16_t distance,
                               SyncDirection direction)
{
    if (master == slave)
    {
        throw std::invalid_argument("A motor can't be synchronized with itself");
    }
    setMotorOutput(master, duty);
    setMotorOutput(slave, direction == SyncDirection::Same ? duty : static_cast<std::int16_t>(-duty));

    // The slave's entry names the master (index + 1). Both get the same distance: if they differ, the pair stops at
    // the shorter one and the other never reports reaching its target.
    output.motorMasterValues.at(slave) = static_cast<std::uint8_t>(master + 1);
    for (std::size_t motor : {master, slave})
    {
        output.motorDistanceValues.at(motor) = distance;
        ++output.motorCommandId.at(motor);
    }
}

void Extension::stopMotor(std::size_t motor)
{
    // A motor that finished its distance command stands already; a new command would only reset the counters.
    auto finished = [&](std::size_t i) { return output.motorDistanceValues.at(i) != 0 && motorCommandDone(i); };
    std::optional<std::size_t> partner = syncPartner(motor);
    if (finished(motor) && (!partner || finished(*partner)))
    {
        output.pwmOutputValues.at(2 * motor) = 0;
        output.pwmOutputValues.at(2 * motor + 1) = 0;
        return;
    }
    setMotorOutput(motor, 0); // a plain motor keeps its counter, a running command is cancelled
}

std::optional<std::size_t> Extension::syncPartner(std::size_t motor) const
{
    for (std::size_t other = 0; other < kMotors; ++other)
    {
        if (other != motor &&
            (output.motorMasterValues[other] == motor + 1 || output.motorMasterValues.at(motor) == other + 1))
        {
            return other;
        }
    }
    return std::nullopt;
}

void Extension::releaseMotor(std::size_t motor)
{
    bool commandRunning = output.motorDistanceValues.at(motor) != 0 || output.motorMasterValues.at(motor) != 0;
    while (std::optional<std::size_t> partner = syncPartner(motor))
    {
        output.stopMotor(*partner); // the firmware would hold it at our counter otherwise; clears its master entry
        if (output.motorMasterValues[motor] == *partner + 1)
        {
            output.motorMasterValues[motor] = 0;
        }
        commandRunning = true;
    }
    if (commandRunning)
    {
        output.motorDistanceValues.at(motor) = 0;
        output.motorMasterValues.at(motor) = 0;
        ++output.motorCommandId.at(motor); // the firmware only takes over the new settings with a new command
    }
}

} // namespace ftrobotx
