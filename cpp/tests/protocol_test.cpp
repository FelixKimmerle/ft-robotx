// Checks the C++ protocol code against byte vectors recorded from the C# implementation.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ftrobotx/Packets.hpp"

using namespace ftrobotx;

namespace
{

int failures = 0;

void check(bool ok, const std::string& what, const std::string& actual = {}, const std::string& expected = {})
{
    if (!ok)
    {
        ++failures;
        std::cerr << "FAIL: " << what << "\n  actual:   " << actual << "\n  expected: " << expected << '\n';
    }
}

std::string hex(const std::vector<std::uint8_t>& bytes)
{
    std::string result;
    char buffer[3];
    for (std::uint8_t b : bytes)
    {
        std::snprintf(buffer, sizeof(buffer), "%02X", b);
        result += buffer;
    }
    return result;
}

std::string hex(const std::string& text)
{
    return hex(std::vector<std::uint8_t>(text.begin(), text.end()));
}

std::vector<std::uint8_t> unhex(const std::string& text)
{
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i + 1 < text.size(); i += 2)
    {
        bytes.push_back(static_cast<std::uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

void checkPacket(Packet& packet, const std::string& name, const std::string& expected)
{
    packet.setSessionId(0xABCD);
    packet.setTransactionId(0x0102);
    std::string actual = hex(packet.bytes());
    check(actual == expected, name, actual, expected);
}

std::string dumpInput(const Input& x)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < kUniversalInputs; ++i)
    {
        out << x.universalInput(i) << ',';
    }
    for (std::size_t i = 0; i < kMotors; ++i)
    {
        out << +x.counterInput(i) << ',' << x.counterValue(i) << ',' << x.resetCounter(i) << ','
            << x.motorReachedDestination(i) << ',';
    }
    out << x.displayButtonLeft() << ',' << x.displayButtonRight() << ',' << (x.changed ? "True" : "False");
    return out.str();
}

} // namespace

int main()
{
    std::vector<Extension> ex;
    ex.emplace_back(0);
    ex.emplace_back(1);
    ex[1].config.setMotorConfig(2, 0);
    ex[0].config.setCounterConfig(3, 7);
    ex[0].config.setUniversalInputConfig(4, {InputMode::Ultrasonic, false});
    ex[1].config.setUniversalInputConfig(1, {InputMode::Voltage, true});
    Output& o = ex[1].output;
    o.motorMasterValues[0] = 3;
    o.pwmOutputValues[4] = 512;
    o.pwmOutputValues[1] = -300;
    o.motorDistanceValues[0] = 200;
    o.motorCommandId[0] = 65535;
    o.counterResetCommandId[2] = 9;

    EchoPacket echo;
    checkPacket(echo, "EchoPacket", "0255001402000000010000000201CDAB0100000001000000FE6C03");
    ConfigPacket config(ex);
    checkPacket(config, "ConfigPacket",
                "0255007C02000000010000000201CDAB0500000002000000000000000101010181818181038181810101010700000000"
                "000000000000000000000000000000000000000000000000000000000100000001010001818081818181818101010101"
                "0000000000000000000000000000000000000000000000000000000000000000F65803");
    RequestInfoPacket info(ex);
    checkPacket(info, "RequestInfoPacket", "0255001C02000000010000000201CDAB06000000020000000000000001000000FE5D03");
    OutputPacket output(ex);
    checkPacket(output, "OutputPacket",
                "0255007402000000010000000201CDAB0200000002000000000000000000000000000000000000000000000000000000"
                "000000000000000000000000000000000000000000000000010000000000000009000000030000000000D4FE00000000"
                "0002000000000000C800000000000000FFFF000000000000F96303");

    {
        EchoResponse r;
        r.load(unhex("0255001401000000020000003412EFBE65000000010000006E8B09"));
        check(r.success() && r.sessionId() == 48879 && r.transactionId() == 4660, "EchoResponse ok");
        EchoResponse bad;
        bad.load(unhex("0255001401000000020000003412EFBE66000000010000006E8B09"));
        check(!bad.success(), "EchoResponse failure");
    }
    {
        ConfigResponse r(2);
        auto bytes = unhex("0255001C01000000020000003412EFBE69000000020000000000000001000000CFD628");
        check(r.length() == bytes.size(), "ConfigResponse length");
        r.load(bytes);
        check(r.success(), "ConfigResponse success");
    }
    {
        RequestInfoResponse r(ex);
        auto bytes = unhex(
            "0255009C01000000020000003412EFBE6A0000000200000001000000545800000000000000000000000000000005096E676BE7F10D"
            "46D827BAED2722FBB9E4FED667871A85B386A8AF727663A7813769819AADEBB3BF2989FD0051C6000000000E71052D7EEFE9B3E53E"
            "429BCF758038B5BEA0600BF84A173EFD1D8462C009CA918154357B0AEA92B1FA06747C8EB987801810E04480EA1527EEEE80D264"
            "5125357709");
        check(r.length() == bytes.size(), "RequestInfoResponse length");
        r.load(bytes);
        check(hex(ex[0].name) == "0E71052D7E3F3F3F3F3E423F3F753F383F", "name 0", hex(ex[0].name));
        check(ex[0].version && ex[0].version->toString() == "234.21" && ex[0].version->dllVersion == 128 &&
                  ex[0].version->transferAreaMajor == 238 && ex[0].version->transferAreaMinor == 128,
              "version 0");
        check(hex(ex[0].macAddress) == "3F3F600B3F4A173E3F1D3F623F093F3F3F54", "mac 0", hex(ex[0].macAddress));
        check(ex[1].name == "TX", "name 1", ex[1].name, "TX");
        check(ex[1].version && ex[1].version->toString() == "173.235" && ex[1].version->dllVersion == 154 &&
                  ex[1].version->transferAreaMajor == 41 && ex[1].version->transferAreaMinor == 137,
              "version 1");
        check(hex(ex[1].macAddress) == "05096E676B3F3F0D463F273F3F27223F3F3F", "mac 1", hex(ex[1].macAddress));
    }
    {
        InputResponse r(ex);
        auto bytes = unhex(
            "0255007C01000000020000003412EFBE660000000200000001000000579AB036CFD6280BB3C707DBAEF270DC9505096E676BE7F1"
            "0D46D827BAED2722FBB9E4FED667871A85B386A8AF72766300000000819AADEBB3BF2989FD0051C69FE4E0F50E71052D7EEFE9B3"
            "E53E429BCF758038B5BEA0600BF84A173EFD1D8462C009CA918154");
        check(r.length() == bytes.size(), "InputResponse length");
        r.load(bytes);
        std::string expected0 = "39553,60333,49075,35113,253,50769,58527,62944,14,61310,48821,64830,113,46057,24736,"
                                "33821,5,16101,63499,49250,45,39746,5962,51721,30159,14464,True";
        std::string expected1 = "39511,14000,54991,2856,51123,56071,62126,56432,149,27495,47611,45957,5,61927,65252,"
                                "43142,9,17933,26582,29359,110,10200,6791,25462,60858,8743,True";
        check(dumpInput(ex[0].input) == expected0, "input 0", dumpInput(ex[0].input), expected0);
        check(dumpInput(ex[1].input) == expected1, "input 1", dumpInput(ex[1].input), expected1);
        check(r.sessionId() == 48879 && r.transactionId() == 4660, "InputResponse ids");
    }

    if (failures == 0)
    {
        std::cout << "All protocol checks passed\n";
    }
    return failures == 0 ? 0 : 1;
}
