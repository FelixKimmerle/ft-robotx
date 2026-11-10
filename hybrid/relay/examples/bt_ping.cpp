// Bluetooth messages between ROBO TX Controllers: round trip time and messages
// per second. Build: hybrid/relay (see README.md). Usage: ftrobotx_bt_ping [-p
// PORT] [-r PORT] [-n COUNT]; --help explains the options.
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION                                                            \
  "Sends Bluetooth messages from one ROBO TX Controller to another (TX <-> "   \
  "TX, not PC <-> TX) and measures\n"                                          \
  "them. With -r, the second controller listens and sends every message "      \
  "back; both are driven from this\n"                                          \
  "program in online mode, so its round trip includes the PC's polling of "    \
  "the second controller. Without -r,\n"                                       \
  "the controller connects to itself (loopback). Then a burst: each message "  \
  "is sent once the previous one is\n"                                         \
  "confirmed, which gives the messages per second. With -a, the second "       \
  "controller runs its own program\n"                                          \
  "(hybrid/relay/tx/bt_echo.c) and only the first one talks to the PC. "       \
  "Ctrl+C stops."

namespace {

using Clock = std::chrono::steady_clock;
using namespace ftrobotx;

constexpr int kChannel = 1;
constexpr auto kConnectTimeout = std::chrono::seconds(20);
constexpr auto kReplyTimeout = std::chrono::seconds(5);

volatile std::sig_atomic_t stopRequested = 0;

void onSignal(int /*signal*/) { stopRequested = 1; }

BtAddress addressOf(const Extension &controller) {
  std::optional<BtAddress> address = parseBtAddress(controller.macAddress);
  if (!address) {
    throw std::runtime_error(
        "Controller reports no valid Bluetooth address: '" +
        controller.macAddress + "'");
  }
  return *address;
}

void print(const char *who, const BtEvent &event) {
  std::printf("  %s: %s on channel %d: %s\n", who, toString(event.command),
              event.channel, toString(event.status));
}

double milliseconds(Clock::duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

} // namespace

int main(int argc, char *argv[]) {
  std::string portOption;
  std::string remoteOption;
  std::string echoOption;
  std::string countOption = "20";
  int exitCode = 0;
  if (!cli::parse(argc, argv, "[-p PORT] [-r PORT | -a ADDRESS] [-n COUNT]",
                  DESCRIPTION,
                  {{"-p", "--port", "PORT", cli::kPortHelp, &portOption},
                   {"-r", "--remote", "PORT",
                    "Port of the second controller, which answers (e.g. "
                    "bt:00:13:7B:48:B2:E2).\n"
                    "                        Default: none, the first "
                    "controller talks to itself.",
                    &remoteOption},
                   {"-a", "--address", "ADDRESS",
                    "Bluetooth address of a second controller running BtEcho "
                    "(hybrid/relay/tx),\n"
                    "                        e.g. 00:13:7B:48:B2:E2.",
                    &echoOption},
                   {"-n", "--count", "COUNT",
                    "Number of pings and of burst messages. Default: 20.",
                    &countOption}},
                  nullptr, exitCode)) {
    return exitCode;
  }

  try {
    int count = std::stoi(countOption);
    std::string portName = cli::choosePort(portOption);
    if (portName.empty() || count < 1) {
      return portName.empty() ? 1 : 2;
    }
    std::signal(SIGINT, onSignal);

    FtConnection sender(portName);
    std::unique_ptr<FtConnection> remote;
    if (!remoteOption.empty()) {
      std::cout << "Connecting to the second controller on " << remoteOption
                << "...\n";
      remote = std::make_unique<FtConnection>(remoteOption);
    }
    FtConnection &receiver = remote ? *remote : sender;
    BtAddress senderAddress = addressOf(sender.extensions().front());
    BtAddress receiverAddress = addressOf(receiver.extensions().front());
    std::string receiverName = receiver.extensions().front().name;
    bool echoProgram = !echoOption.empty();
    if (echoProgram) {
      if (remote) {
        std::cerr << "Use either -r or -a\n";
        return 2;
      }
      std::optional<BtAddress> address = parseBtAddress(echoOption);
      if (!address) {
        std::cerr << "Invalid Bluetooth address " << echoOption << '\n';
        return 2;
      }
      receiverAddress = *address;
      receiverName = "BtEcho program";
    }
    std::cout << "Sender " << sender.extensions().front().name << " ("
              << toString(senderAddress) << "), receiver " << receiverName
              << " (" << toString(receiverAddress) << ")"
              << (remote || echoProgram ? "" : ", loopback") << '\n';

    // Events of both sides; the receiver echoes every message back.
    bool connected = false;
    bool receiverConnected =
        !remote; // loopback: one connection, both ends on the sender
    bool failed = false;
    int sendsConfirmed = 0;
    std::vector<std::vector<std::uint8_t>> received; // at the sender
    int receivedByRemote = 0;
    bool echo = true; // the receiver sends messages back; in the burst it only
                      // counts them

    sender.setBtCallback([&](const BtEvent &event) {
      if (event.status == BtStatus::MessageIndication) {
        received.push_back(event.message);
        return;
      }
      if (event.command == BtCommand::Send) {
        sendsConfirmed += event.status == BtStatus::Success ? 1 : 0;
        failed |= event.status != BtStatus::Success;
        return;
      }
      print("sender", event);
      if (event.command == BtCommand::Connect) {
        connected = event.status == BtStatus::Success ||
                    event.status == BtStatus::ConnectionExists;
        failed |= !connected;
      }
    });
    if (remote) {
      remote->setBtCallback([&](const BtEvent &event) {
        if (event.status == BtStatus::MessageIndication) {
          ++receivedByRemote;
          if (echo) {
            remote->btSend(kChannel, event.message);
          }
          return;
        }
        if (event.command == BtCommand::Send) {
          return;
        }
        print("receiver", event);
        if (event.command == BtCommand::Listen &&
            event.status == BtStatus::ConnectionIndication) {
          receiverConnected = true;
          remote->btStartReceive(kChannel);
        }
      });
    }

    auto updateBoth = [&] {
      sender.update();
      if (remote) {
        remote->update();
      }
    };
    auto waitFor = [&](const std::function<bool()> &done,
                       Clock::duration timeout) {
      auto end = Clock::now() + timeout;
      while (!done() && !failed && stopRequested == 0 && Clock::now() < end) {
        updateBoth();
      }
      return done();
    };

    // Leftovers of an earlier run would make connect report "already connected"
    // or "channel busy".
    sender.btDisconnect(kChannel);
    if (remote) {
      remote->btStopListen(kChannel);
      remote->btDisconnect(kChannel);
      remote->btListen(kChannel, senderAddress);
    }
    waitFor([] { return false; }, std::chrono::milliseconds(500));

    std::cout << "Connecting (takes a few seconds)...\n";
    auto connectStart = Clock::now();
    sender.btConnect(kChannel, receiverAddress);
    if (!waitFor([&] { return connected && receiverConnected; },
                 kConnectTimeout)) {
      std::cerr << "No connection\n";
      return 1;
    }
    std::printf("Connected after %.0f ms\n",
                milliseconds(Clock::now() - connectStart));
    sender.btStartReceive(kChannel);
    waitFor([] { return false; }, std::chrono::milliseconds(300));

    // Ping: one message at a time, wait for it to come back.
    std::vector<double> roundTrips;
    for (int i = 0; i < count && stopRequested == 0; ++i) {
      std::string text = "ping " + std::to_string(i);
      received.clear();
      auto start = Clock::now();
      sender.btSend(kChannel, text);
      if (!waitFor([&] { return !received.empty(); }, kReplyTimeout)) {
        std::printf("ping %d: no answer\n", i);
        continue;
      }
      roundTrips.push_back(milliseconds(Clock::now() - start));
      std::string answer(received.front().begin(), received.front().end());
      if (answer != text) {
        std::printf("ping %d: wrong answer '%s'\n", i, answer.c_str());
      }
    }
    if (!roundTrips.empty()) {
      double sum = 0;
      double low = roundTrips.front();
      double high = roundTrips.front();
      for (double t : roundTrips) {
        sum += t;
        low = std::min(low, t);
        high = std::max(high, t);
      }
      std::printf("Round trip: %zu of %d answered, average %.1f ms, min %.1f "
                  "ms, max %.1f ms\n",
                  roundTrips.size(), count,
                  sum / static_cast<double>(roundTrips.size()), low, high);
    }

    // Burst: the next message as soon as the previous one is confirmed; 16
    // bytes each. With a second controller it only counts them, so this is the
    // rate from one controller to the other.
    received.clear();
    receivedByRemote = 0;
    echo = !remote;
    sendsConfirmed = 0;
    auto burstStart = Clock::now();
    for (int i = 0; i < count && stopRequested == 0; ++i) {
      char text[17];
      std::snprintf(text, sizeof(text), "burst %9d", i);
      sender.btSend(kChannel, std::string_view(text, 16));
      if (!waitFor([&] { return sendsConfirmed > i; }, kReplyTimeout)) {
        std::printf("burst: message %d not confirmed\n", i);
        break;
      }
    }
    double burstSeconds =
        std::chrono::duration<double>(Clock::now() - burstStart).count();
    auto arrived = [&] {
      return remote ? receivedByRemote : static_cast<int>(received.size());
    };
    waitFor([&] { return arrived() >= sendsConfirmed; },
            std::chrono::seconds(3));
    std::printf("Burst: %d messages confirmed in %.2f s = %.1f messages/s "
                "(%.0f bytes/s); %d arrived\n",
                sendsConfirmed, burstSeconds, sendsConfirmed / burstSeconds,
                16 * sendsConfirmed / burstSeconds, arrived());

    sender.btDisconnect(kChannel);
    if (remote) {
      remote->btStopListen(kChannel);
    }
    waitFor([] { return false; }, std::chrono::milliseconds(500));
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
