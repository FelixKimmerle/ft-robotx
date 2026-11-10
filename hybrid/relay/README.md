# Hybrid mode with a relay TX

Hybrid mode: a ROBO TX Controller runs a local program (later a small VM), and the PC talks to it through a second
TX that stays in online mode and relays Bluetooth messages:

```text
PC  --USB or Bluetooth, online mode-->  relay TX  --Bluetooth messages (TX <-> TX)-->  model TX (local program)
```

A local program can't talk to the PC itself: online-mode packets interrupt it, and the firmware gives programs no
USB or serial API. What programs do have are Bluetooth messages to other ROBO TX Controllers, and the relay TX can
send and receive those on behalf of the PC in online mode. The relay keeps its own inputs and outputs usable at the
same time. (`hybrid/direct/` is reserved for a later attempt without the relay, the PC speaking the TX-to-TX
protocol itself.)

Status: the online-mode commands for Bluetooth messages are reverse engineered and in the libraries (C++ and C#),
tested with two controllers. On the model TX, an echo program ([`tx/bt_echo.c`](tx/bt_echo.c)) answers messages
for measurements. The command protocol and the VM don't exist yet.

## Using it from the libraries

C++ (`#include <ftrobotx/FtConnection.hpp>`):

```cpp
FtConnection relay("/dev/ttyACM0");
BtAddress model = *parseBtAddress("00:13:7B:48:B2:E2");

relay.setBtCallback([&](const BtEvent& event) {
    if (event.command == BtCommand::Connect && event.status == BtStatus::Success)
        relay.btStartReceive(1);                   // only possible once connected
    if (event.status == BtStatus::MessageIndication)
        handle(event.message);                     // up to 16 bytes
});
relay.btConnect(1, model);                         // the model must listen for the relay (btListen / BtListenConOn)
while (running)
{
    relay.update();                                // callbacks come from here
    relay.btSend(1, "M1 512");                     // at most 16 bytes
}
```

C#: `BtConnect`, `BtListen`, `BtStartReceive`, `BtSend`, ..., the events `BtEventReceived` and
`BtScanResultReceived`, `BtAddress.Parse`.

Commands only send the request. Results and everything the controller reports by itself come through the callback
(C#: events) from `update()`. Connecting to the controller's own address gives a loopback connection: messages
come straight back, good for testing with one controller.

## Example: `ftrobotx_bt_ping`

[`examples/bt_ping.cpp`](examples/bt_ping.cpp) sends numbered messages from one TX to another and back, measures the
round trip, then sends a burst to measure messages per second. The second TX can answer in three ways:

- `-a ADDRESS`: it runs the echo program [`tx/bt_echo.c`](tx/bt_echo.c), the real hybrid setup (only the relay
  talks to the PC).
- `-r PORT`: it is driven from the PC in online mode too, and the PC echoes the messages.
- neither: the first TX talks to itself (loopback).

```bash
cd hybrid/relay && cmake -S . -B build -G Ninja && ninja -C build
```

```bash
./build/ftrobotx_bt_ping                             # loopback on the TX found on USB
```

```bash
./build/ftrobotx_bt_ping -r bt:00:13:7B:48:B2:E2     # TX on USB sends, TX-618 (Bluetooth, online) answers
```

This project includes the online library with `add_subdirectory(../../online/cpp)`, the way your own project would.

## Programs for the model TX: `tx/`

C programs for the controller behind the relay, built with the offline SDK (`../../../offline`, ARM cross
toolchain). The model TX only accepts a connection from the relay; set its address at configure time:

```bash
cd hybrid/relay/tx && cmake -S . -B build -G Ninja -DFT_RELAY_ADDRESS=00:13:7B:53:DB:5E && ninja -C build
```

```bash
../../offline/tools/ftrobotx-load --port bt:00:13:7B:48:B2:E2 deploy tx/build/BtEcho.bin   # from hybrid/relay
```

`BtEcho` listens for the relay on channel 1, starts receiving once connected, sends every message back from the
receive callback (as fischertechnik's StopGoBt demo does) and listens again after a disconnect. Messages, failed
sends and the last status are shown on the display. While it runs, don't talk to that TX in online mode: that stops
the program.

## The protocol

Found by disassembling the static debug build of ftMscLib (`FtMscLib_Static_LIBCMTD_Debug.lib`, part of
fischertechnik's *PC Programming ROBO TX Controller* package; its object files still carry the function names, e.g.
`FishX1BtConnectRequest` in `BtConnect.obj`), then verified on two controllers (firmware 1.30).

Bluetooth commands are ordinary online-mode packets (`02 55`, length, header, payload, checksum, `03`) with their own
telecodes. ftMscLib sends them with transaction id 0. The controller answers with telecode + 100.

| Telecode | ftMscLib function | Payload |
| -------- | ----------------- | ------- |
| 10 | `StartScanBtDevice` | none |
| 11 | `CancelScanBtDevice` | none |
| 12 | `ConnectBtAddress` | channel (1 byte), address (6 bytes, as printed: `00 13 7B ...`), 0 |
| 13 | `DisconnectBt` | channel, 0, 0, 0 |
| 14 | `SendBtMessage` | channel, 0, length (16 bit, little endian), message, 0, 0 |
| 15 | `BtReadMsgOn` | channel, 0, 0, 0 |
| 16 | `BtReadMsgOff` | channel, 0, 0, 0 |
| 17 | `BtListenConOn` | channel, address **of the remote controller**, 0 |
| 18 | `BtListenConOff` | channel, 0, 0, 0 |

Replies (telecode + 100), after the count field:

- 112 ... 118: `BT_CB`: channel (16 bit), status (16 bit, `CB_BtStatus` in `ROBO_TX_FW.h`, the same in ftMscLib).
- 115 with status 13 (`BT_MSG_INDICATION`): `BT_RECV_CB`: channel, status, length (16 bit), message (16 bytes).
- 110, 111: `BT_SCAN_STATUS`: status (16 bit, `BtInquiryScanStatus`), address (6 bytes), 2 bytes padding, name.

**Not every reply comes right away, and some come unasked**:

- Connect (12) answers only once the connection is up or failed, 2–8 s later (loopback: at once).
- A scan answers at once with "started", then sends one packet per device found (the controller's own address
  first) and "ended" after about 20 s.
- Once receiving is switched on, every incoming message comes as a 115 packet. A listening channel reports an
  incoming connection (117, status 11) and its end (117, status 12).

So the libraries read whole packets by their length field, queue every telecode 110 ... 118 as an event, and only
the expected reply ends the wait for a normal online-mode command. Events reach the callbacks from `update()`.

**One request at a time.** If the next packet follows a Bluetooth command right away, the controller sends parts of
it back (garbage on the line) and may drop it, which shows up as a lost reply. ftMscLib handles one request at a
time, and so do the libraries: after a Bluetooth command they wait for its immediate reply (connect excepted, which
has none). Under heavy traffic (hundreds of received messages per second) the controller still loses an online
reply or parts of it now and then; the libraries skip the broken bytes and send the packet again. That is safe
because every online packet carries state (outputs, configuration, command ids).

### Setting up a connection

1. Receiver: listen (17) with the **sender's** address. Without it, a connection attempt fails with status 16
   (`BT_NO_LISTEN_ACTIVE`): a TX doesn't accept connections by default.
2. Sender: connect (12) with the receiver's address. Reply after a few seconds: status 0. The receiver reports 117
   with status 11.
3. Both sides: read on (15), **after** the connection is up; before, it answers status 7 (no connection). Messages
   that arrive without it are dropped by the firmware.
4. Send (14); the reply 114 with status 0 confirms it.

Connections and the receive setting survive the end of the online session: a later run may get "already connected"
or "already receiving". Disconnect / stop listening first.

## Measurements

Firmware 1.30. TX-A (relay) on USB as sender, ROBO TX-618 as receiver, 2026-11-10.

| What | Result |
| ---- | ------ |
| Largest message | **16 bytes.** Longer messages (the firmware takes up to 255) arrive cut to their first 16 bytes, not split. The libraries refuse longer ones. |
| Connect | 1.7–3.7 s |
| **TX-618 runs BtEcho** (`-a`): round trip PC → relay → model → relay → PC | **average 23–24 ms**, 17–48 ms; about 1 in 50–300 pings lost |
| **TX-618 runs BtEcho**: burst, next message after the send confirmation | **220–240 messages/s** (about 3.7 KB/s), all arrive |
| Loopback, one TX on USB | round trip 3.4 ms, 280–300 messages/s |
| TX-618 driven by the PC in online mode over Bluetooth (`-r`) | round trip 210–450 ms, 2–7 messages/s: the PC polls TX-618 at ~12 cycles/s and its radio carries both links |

Before the libraries waited for each Bluetooth command's reply, confirmations were lost (about 1 in 12–40) and
online replies timed out; most of that came from sending too early (see above). Sustained bursts of several hundred
messages per second can still break the TX-to-TX connection without a disconnect report (every send then answers
status 7) and leave the echo program needing a restart. Normal traffic is far below that.

What this means for the hybrid protocol:

- **Frames of at most 16 bytes.** A program upload needs fragmentation: sequence number, fragment index, checksum.
- **Our own sequence numbers, acknowledgements and resends**, not the firmware's send confirmation alone; plus a
  watchdog on both sides, since a connection can die silently.
- **Commands at ~20 ms round trip** are fine for setting outputs and reading values from the PC (and for XCP), but
  not for closed-loop control of a fast line follower. That runs on the model TX (the VM); the PC sends commands,
  programs and parameters, the model sends values when they change or at a requested rate.

## Still unknown

- Telecodes 19 and 20 (answer with 8 bytes, probably status queries such as `StatusBtConnection`), and 3 and 4
  (answer only with a payload; maybe I2C or memory layout). The other objects in the static library (`I2cApi.obj`,
  `ftRemote.obj`, `ftFileTrans.obj`) describe I2C, the remote shell and file transfer the same way.
- Whether the relay keeps up while the PC also polls its inputs at full rate.
