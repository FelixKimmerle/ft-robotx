#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ftrobotx/Bluetooth.hpp"
#include "ftrobotx/Extension.hpp"

namespace ftrobotx
{

/// Appends integers in a fixed byte order, independent of the host.
class ByteWriter
{
  public:
    void u8(std::uint8_t value)
    {
        bytes_.push_back(value);
    }

    template <typename T> void le(T value)
    {
        for (std::size_t i = 0; i < sizeof(T); ++i)
        {
            bytes_.push_back(byteAt(value, i));
        }
    }

    template <typename T> void be(T value)
    {
        for (std::size_t i = sizeof(T); i-- > 0;)
        {
            bytes_.push_back(byteAt(value, i));
        }
    }

    void append(std::span<const std::uint8_t> bytes)
    {
        bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
    }

    [[nodiscard]] const std::vector<std::uint8_t>& bytes() const
    {
        return bytes_;
    }
    [[nodiscard]] std::vector<std::uint8_t> take()
    {
        return std::move(bytes_);
    }

  private:
    template <typename T> static std::uint8_t byteAt(T value, std::size_t i)
    {
        return static_cast<std::uint8_t>(static_cast<std::uint64_t>(value) >> (8 * i));
    }

    std::vector<std::uint8_t> bytes_;
};

class Header
{
  public:
    /// Size of the header on the wire, including the 0x02 0x55 start bytes.
    static constexpr std::size_t kSize = 24;

    std::int32_t from = 0x02;
    std::int32_t to = 0x01;
    std::uint16_t transactionId = 0;
    std::uint16_t sessionId = 0;
    std::int32_t commandCode = 0;
    std::int32_t numPayload = 1;
    std::int16_t length = 27;

    void load(std::span<const std::uint8_t> bytes);

    /// Writes the header without the start bytes.
    void write(ByteWriter& writer) const;
};

class Packet
{
  public:
    virtual ~Packet() = default;

    [[nodiscard]] std::int32_t commandCode() const
    {
        return header_.commandCode;
    }

    void setSessionId(std::uint16_t id)
    {
        header_.sessionId = id;
    }
    void setTransactionId(std::uint16_t id)
    {
        header_.transactionId = id;
    }

    [[nodiscard]] std::vector<std::uint8_t> bytes();

  protected:
    virtual void writePayload(ByteWriter& /*writer*/) const
    {
    }

    Header header_;
    bool sendPayload_ = true;
};

class Response
{
  public:
    /// Length of a response without payload: header, CRC and end byte.
    static constexpr std::size_t kMinLength = 27;

    virtual ~Response() = default;

    /// Total number of bytes this response occupies on the wire.
    [[nodiscard]] virtual std::size_t length() const
    {
        return kMinLength;
    }

    void load(std::span<const std::uint8_t> bytes);

    [[nodiscard]] std::uint16_t sessionId() const
    {
        return header_.sessionId;
    }
    [[nodiscard]] std::uint16_t transactionId() const
    {
        return header_.transactionId;
    }
    [[nodiscard]] std::int32_t commandCode() const
    {
        return header_.commandCode;
    }

  protected:
    virtual void loadPayload(std::span<const std::uint8_t> /*payload*/)
    {
    }

    Header header_;
};

class EchoPacket : public Packet
{
  public:
    EchoPacket();
};

class EchoResponse : public Response
{
  public:
    [[nodiscard]] bool success() const
    {
        return header_.commandCode == 0x65;
    }
};

class ConfigPacket : public Packet
{
  public:
    explicit ConfigPacket(const std::vector<Extension>& extensions);

  protected:
    void writePayload(ByteWriter& writer) const override;

  private:
    const std::vector<Extension>& extensions_;
};

class ConfigResponse : public Response
{
  public:
    explicit ConfigResponse(std::size_t numExtensions) : numExtensions_(numExtensions)
    {
    }

    [[nodiscard]] std::size_t length() const override
    {
        return kMinLength + 4 * numExtensions_;
    }
    [[nodiscard]] bool success() const
    {
        return header_.commandCode == 0x69;
    }

  private:
    std::size_t numExtensions_;
};

class RequestInfoPacket : public Packet
{
  public:
    explicit RequestInfoPacket(const std::vector<Extension>& extensions);

  protected:
    void writePayload(ByteWriter& writer) const override;

  private:
    const std::vector<Extension>& extensions_;
};

class RequestInfoResponse : public Response
{
  public:
    explicit RequestInfoResponse(std::vector<Extension>& extensions) : extensions_(extensions)
    {
    }

    [[nodiscard]] std::size_t length() const override
    {
        return kMinLength + 68 * extensions_.size();
    }

  protected:
    void loadPayload(std::span<const std::uint8_t> payload) override;

  private:
    std::vector<Extension>& extensions_;
};

/// Shows a text on a controller's display, or clears it with an empty text (telecode 8, like ftMscLib's
/// SetRoboTxMessage). The firmware only shows a message whose id differs from the previous one.
class MessagePacket : public Packet
{
  public:
    /// Longest text the firmware shows.
    static constexpr std::size_t kMaxLength = 98;

    MessagePacket(std::uint32_t transferAreaIndex, std::uint8_t messageId, std::string_view text);

  protected:
    void writePayload(ByteWriter& writer) const override;

  private:
    std::uint32_t transferAreaIndex_;
    std::uint8_t messageId_;
    std::string text_;
};

class MessageResponse : public Response
{
  public:
    [[nodiscard]] std::size_t length() const override
    {
        return kMinLength + 4;
    }
    [[nodiscard]] bool success() const
    {
        return header_.commandCode == 108;
    }
};

/// Reads the master's state (telecode 7, ftMscLib's STATE_RD_REQUEST). Of the 24 bytes per controller, only the
/// first 8 are known: whether extension 1 ... 8 is connected.
class StatePacket : public Packet
{
  public:
    StatePacket();

  protected:
    void writePayload(ByteWriter& writer) const override;
};

class StateResponse : public Response
{
  public:
    [[nodiscard]] std::size_t length() const override
    {
        return kMinLength + 4 + 24;
    }
    [[nodiscard]] bool success() const
    {
        return header_.commandCode == 107;
    }
    /// Extension 1 ... 8 connected to the master
    [[nodiscard]] const std::array<bool, 8>& extensionsOnline() const
    {
        return extensionsOnline_;
    }

  protected:
    void loadPayload(std::span<const std::uint8_t> payload) override;

  private:
    std::array<bool, 8> extensionsOnline_{};
};

class OutputPacket : public Packet
{
  public:
    explicit OutputPacket(const std::vector<Extension>& extensions);

  protected:
    void writePayload(ByteWriter& writer) const override;

  private:
    const std::vector<Extension>& extensions_;
};

class InputResponse : public Response
{
  public:
    explicit InputResponse(std::vector<Extension>& extensions) : extensions_(extensions)
    {
    }

    [[nodiscard]] std::size_t length() const override
    {
        return kMinLength + 52 * extensions_.size();
    }

  protected:
    void loadPayload(std::span<const std::uint8_t> payload) override;

  private:
    std::vector<Extension>& extensions_;
};

/// Bluetooth message commands (telecodes 10 ... 18, found in ftMscLib). The controller answers with telecode + 100,
/// some right away, some seconds later (connect), and also sends such packets by itself (received messages, incoming
/// connections, scan results); FtConnection treats all of them as events.
class BtPacket : public Packet
{
  public:
    static constexpr std::int32_t kScan = 10;
    static constexpr std::int32_t kCancelScan = 11;
    static constexpr std::int32_t kConnect = 12;
    static constexpr std::int32_t kDisconnect = 13;
    static constexpr std::int32_t kSend = 14;
    static constexpr std::int32_t kStartReceive = 15;
    static constexpr std::int32_t kStopReceive = 16;
    static constexpr std::int32_t kListen = 17;
    static constexpr std::int32_t kStopListen = 18;

    /// Whether a reply telecode belongs to a Bluetooth command.
    [[nodiscard]] static bool isReply(std::int32_t commandCode)
    {
        return commandCode >= kScan + 100 && commandCode <= kStopListen + 100;
    }

    /// Scan or cancel scan: no payload.
    explicit BtPacket(std::int32_t commandCode);
    /// Disconnect, start/stop receive, stop listen: channel, 0, 0, 0.
    BtPacket(std::int32_t commandCode, int channel);
    /// Connect, listen: channel, address, 0.
    BtPacket(std::int32_t commandCode, int channel, const BtAddress& address);
    /// Send: channel, 0, length (16 bit), message, 2 bytes padding (as ftMscLib does).
    BtPacket(int channel, std::span<const std::uint8_t> message);

  protected:
    void writePayload(ByteWriter& writer) const override;

  private:
    std::vector<std::uint8_t> payload_;
};

/// Any packet with a Bluetooth reply telecode (110 ... 118).
class BtReply : public Response
{
  public:
    /// Scan replies (110, 111) carry a scan status, the others a channel event.
    [[nodiscard]] bool isScan() const
    {
        return header_.commandCode == BtPacket::kScan + 100 || header_.commandCode == BtPacket::kCancelScan + 100;
    }
    [[nodiscard]] const BtEvent& event() const
    {
        return event_;
    }
    [[nodiscard]] const BtScanResult& scanResult() const
    {
        return scanResult_;
    }

  protected:
    void loadPayload(std::span<const std::uint8_t> payload) override;

  private:
    BtEvent event_;
    BtScanResult scanResult_;
};

} // namespace ftrobotx
