#include "ftrobotx/Packets.hpp"

#include <algorithm>
#include <string>

#include "ftrobotx/Errors.hpp"

namespace ftrobotx
{

namespace
{

template <typename T>
T readLE(std::span<const std::uint8_t> bytes, std::size_t offset)
{
    if (offset + sizeof(T) > bytes.size())
    {
        throw ProtocolError("Response too short");
    }
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
    {
        value |= static_cast<std::uint64_t>(bytes[offset + i]) << (8 * i);
    }
    return static_cast<T>(value);
}

template <typename T>
T read(std::span<const std::uint8_t> bytes, std::size_t& offset)
{
    T value = readLE<T>(bytes, offset);
    offset += sizeof(T);
    return value;
}

/// Zero-padded ASCII; like .NET's Encoding.ASCII, non-ASCII bytes become '?'.
std::string readString(std::span<const std::uint8_t> bytes)
{
    auto end = std::find_if(bytes.rbegin(), bytes.rend(), [](std::uint8_t b) { return b != 0; }).base();
    std::string result;
    for (auto it = bytes.begin(); it != end; ++it)
    {
        result.push_back(*it < 0x80 ? static_cast<char>(*it) : '?');
    }
    return result;
}

Extension& extensionAt(std::vector<Extension>& extensions, std::int32_t transferArea)
{
    if (transferArea < 0 || static_cast<std::size_t>(transferArea) >= extensions.size())
    {
        throw ProtocolError("Unknown transfer area " + std::to_string(transferArea));
    }
    return extensions[static_cast<std::size_t>(transferArea)];
}

} // namespace

void Header::load(std::span<const std::uint8_t> bytes)
{
    if (bytes.size() < Response::kMinLength)
    {
        throw ProtocolError("Response too short");
    }
    length = static_cast<std::int16_t>((bytes[2] << 8) | bytes[3]);
    from = readLE<std::int32_t>(bytes, 4);
    to = readLE<std::int32_t>(bytes, 8);
    transactionId = readLE<std::uint16_t>(bytes, 12);
    sessionId = readLE<std::uint16_t>(bytes, 14);
    commandCode = readLE<std::int32_t>(bytes, 16);
    if (bytes.size() > Response::kMinLength)
    {
        numPayload = readLE<std::int32_t>(bytes, 20);
    }
}

void Header::write(ByteWriter& writer) const
{
    writer.be(length);
    writer.le(from);
    writer.le(to);
    writer.le(transactionId);
    writer.le(sessionId);
    writer.le(commandCode);
    writer.le(numPayload);
}

std::vector<std::uint8_t> Packet::bytes()
{
    ByteWriter payload;
    writePayload(payload);

    header_.length = static_cast<std::int16_t>(20 + payload.bytes().size());

    ByteWriter writer;
    writer.u8(0x02);
    writer.u8(0x55);
    header_.write(writer);
    if (sendPayload_)
    {
        writer.append(payload.bytes());
    }

    // Two's complement of the 16-bit sum of all bytes after the start bytes.
    std::uint16_t sum = 0;
    for (std::size_t i = 2; i < writer.bytes().size(); ++i)
    {
        sum = static_cast<std::uint16_t>(sum + writer.bytes()[i]);
    }
    writer.be(static_cast<std::uint16_t>(-sum));
    writer.u8(0x03);

    return writer.take();
}

void Response::load(std::span<const std::uint8_t> bytes)
{
    header_.load(bytes);
    if (bytes.size() > kMinLength)
    {
        if (header_.length < 20 || Header::kSize + static_cast<std::size_t>(header_.length - 20) > bytes.size())
        {
            throw ProtocolError("Invalid length in response header");
        }
        loadPayload(bytes.subspan(Header::kSize, static_cast<std::size_t>(header_.length - 20)));
    }
    else
    {
        loadPayload({});
    }
}

EchoPacket::EchoPacket()
{
    header_.commandCode = 0x01;
    sendPayload_ = false;
}

ConfigPacket::ConfigPacket(const std::vector<Extension>& extensions) : extensions_(extensions)
{
    header_.commandCode = 0x05;
    header_.numPayload = static_cast<std::int32_t>(extensions.size());
}

void ConfigPacket::writePayload(ByteWriter& writer) const
{
    for (const Extension& extension : extensions_)
    {
        writer.le(extension.transferAreaIndex);

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            writer.u8(extension.config.motorConfig(i));
        }

        for (std::size_t i = 0; i < kUniversalInputs; ++i)
        {
            writer.u8(extension.config.universalInputConfig(i).toByte());
        }

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            writer.u8(extension.config.counterConfig(i));
        }

        // Reserved
        for (int i = 0; i < 32; ++i)
        {
            writer.u8(0);
        }
    }
}

RequestInfoPacket::RequestInfoPacket(const std::vector<Extension>& extensions) : extensions_(extensions)
{
    header_.commandCode = 0x06;
    header_.numPayload = static_cast<std::int32_t>(extensions.size());
}

void RequestInfoPacket::writePayload(ByteWriter& writer) const
{
    for (const Extension& extension : extensions_)
    {
        writer.le(extension.transferAreaIndex);
    }
}

void RequestInfoResponse::loadPayload(std::span<const std::uint8_t> payload)
{
    if (payload.size() < 68 * extensions_.size())
    {
        throw ProtocolError("Info response too short");
    }

    std::size_t offset = 0;
    for (std::size_t i = 0; i < extensions_.size(); ++i)
    {
        Extension& extension = extensionAt(extensions_, read<std::int32_t>(payload, offset));

        extension.name = readString(payload.subspan(offset, 17));
        offset += 17;
        extension.macAddress = readString(payload.subspan(offset, 18));
        offset += 18;
        offset += 1;
        // ta_array_start_addr, pgm_area_start_addr, pgm_area_size
        offset += 3 * 4;

        // FT_VERSION: hardware, firmware, transfer area, reserved; each FT_VER is the bytes a, b, c, d
        std::span<const std::uint8_t> version = payload.subspan(offset, 16);
        extension.version = FirmwareVersion{
            .hardwareRevision = static_cast<char>(version[0]),
            .firmwareMajor = version[6],
            .firmwareMinor = version[7],
            .dllVersion = version[5],
            .transferAreaMajor = version[10],
            .transferAreaMinor = version[11],
        };
        offset += 16;
    }
}

OutputPacket::OutputPacket(const std::vector<Extension>& extensions) : extensions_(extensions)
{
    header_.commandCode = 0x02;
    header_.numPayload = static_cast<std::int32_t>(extensions.size());
}

void OutputPacket::writePayload(ByteWriter& writer) const
{
    for (const Extension& extension : extensions_)
    {
        const Output& output = extension.output;

        writer.le(extension.transferAreaIndex);

        for (std::uint16_t value : output.counterResetCommandId)
        {
            writer.le(value);
        }

        for (std::uint8_t value : output.motorMasterValues)
        {
            writer.u8(value);
        }

        for (std::int16_t value : output.pwmOutputValues)
        {
            writer.le(value);
        }

        for (std::uint16_t value : output.motorDistanceValues)
        {
            writer.le(value);
        }

        for (std::uint16_t value : output.motorCommandId)
        {
            writer.le(value);
        }
    }
}

void InputResponse::loadPayload(std::span<const std::uint8_t> payload)
{
    std::size_t offset = 0;

    for (std::int32_t e = 0; e < header_.numPayload; ++e)
    {
        Input& input = extensionAt(extensions_, read<std::int32_t>(payload, offset)).input;

        for (std::size_t i = 0; i < kUniversalInputs; ++i)
        {
            input.setUniversalInput(i, read<std::uint16_t>(payload, offset));
        }

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            input.setCounterInput(i, read<std::uint8_t>(payload, offset));
        }

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            input.setCounterValue(i, read<std::uint16_t>(payload, offset));
        }

        input.setDisplayButtonLeft(read<std::uint16_t>(payload, offset));
        input.setDisplayButtonRight(read<std::uint16_t>(payload, offset));

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            input.setResetCounter(i, read<std::uint16_t>(payload, offset));
        }

        for (std::size_t i = 0; i < kMotors; ++i)
        {
            input.setMotorReachedDestination(i, read<std::uint16_t>(payload, offset));
        }
    }
}

} // namespace ftrobotx
