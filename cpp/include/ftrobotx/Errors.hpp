#pragma once

#include <stdexcept>

namespace ftrobotx
{

/// The controller answered with something that doesn't fit the protocol.
class ProtocolError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

/// The controller didn't answer in time.
class TimeoutError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

} // namespace ftrobotx
