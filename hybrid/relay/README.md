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
tested with two controllers. The local program and VM on the model TX don't exist yet.

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
round trip, then sends a burst to measure messages per second. With `-r PORT`, the second TX listens and answers;
both are driven from the PC in online mode. Without `-r`, the first TX talks to itself (loopback).

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

Firmware 1.30. TX-A on USB as sender, ROBO TX-618 as receiver, 2026-11-10.

| What | Result |
| ---- | ------ |
| Largest message | **16 bytes.** Longer messages (the firmware takes up to 255) arrive cut to their first 16 bytes, not split. The libraries refuse longer ones. |
| Connect | 2.2–3.7 s |
| Loopback, one TX on USB | round trip 3.4 ms, 300 messages/s |
| TX-A → TX-618 → TX-A, relayed by the PC | round trip 210–450 ms, average 260–300 ms |
| Burst TX-A → TX-618, next message after the confirmation | 2–7 messages/s |
| Lost confirmations | about 1 in 12–40 messages: the message arrives, but the sender never gets its 114 |

The round trip and the burst rate are worst cases. TX-618 was connected to the PC over Bluetooth online mode at the
same time, so its radio carried both links, and the PC polled it at only ~12 cycles/s. A model TX with a local
program has no PC link. Its rate needs measuring with a local program.

What this means for the hybrid protocol:

- **Frames of at most 16 bytes.** A program upload needs fragmentation: sequence number, fragment index, checksum.
- **Don't wait for the send confirmation.** It is sometimes lost. Use our own sequence numbers and acknowledgements
  with timeouts, and resend.
- A few messages per second through the relay is too slow for closed-loop control from the PC. Control has to run
  on the model TX. The PC sends programs and parameters, the model sends telemetry in batches.

## Still unknown

- Telecodes 19 and 20 (answer with 8 bytes, probably status queries such as `StatusBtConnection`), and 3 and 4
  (answer only with a payload; maybe I2C or memory layout). The other objects in the static library (`I2cApi.obj`,
  `ftRemote.obj`, `ftFileTrans.obj`) describe I2C, the remote shell and file transfer the same way.
- Message rate and latency between two controllers without a PC link (needs the local program).
- Whether the relay keeps up while the PC also polls its inputs at full rate.
