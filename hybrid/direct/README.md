# Hybrid mode without relay (direct)

Goal: the PC talks to a model TX running a local program directly over Bluetooth, speaking the TX-to-TX message
protocol itself, without a relay TX in between (see [`../relay`](../relay/README.md) for the relay version).

Status (2026-11-10): **blocked by the firmware's address check**; needs a Bluetooth adapter whose address can be set.
Nothing is implemented yet.

## What a ROBO TX offers over Bluetooth

SDP (`sdptool browse 00:13:7B:48:B2:E2`): a single service, "Serial Port" (SPP) on **RFCOMM channel 1**, the same one
online mode uses. Device class `0x821F00`, names "ROBO TX-…". So TX-to-TX messages run over the same serial channel
as online mode; the firmware decides by the other side whether a connection is a PC (online mode) or a controller
(messages).

## The address check

Tested with TX-A in online mode (USB), PC adapter `A0:AD:9F:6F:27:E2`:

| Command on the TX | With the PC's address | With a TX's address |
| ----------------- | --------------------- | ------------------- |
| connect (telecode 12) | status 5 `BT_NOT_ROBOTX` at once; the PC, listening on RFCOMM channel 1, sees no connection attempt | success after 2–4 s |
| listen (telecode 17) | status 5 `BT_NOT_ROBOTX` at once | success |
| connect to `00:00:00:00:00:00` | status 5 at once | – |

The answer comes before any radio traffic, so the firmware checks the address itself, most likely its first three
bytes: every ROBO TX address starts with `00:13:7B`. The PC can neither be a message partner nor accept a TX's
connection with its own address.

## Way forward

1. **A Bluetooth adapter with a settable address.** Both adapters on the test PC are ASUS USB dongles `0b05:1bef`
   (Realtek, probably RTL8761B / USB-BT500): neither offers it (`btmgmt --index <n> config`: no supported options).
   The RTL8761B's address can only be patched into its firmware configuration (`rtl8761bu_config.bin`), which applies
   to every dongle of that type on the PC and is overwritten by firmware updates, so not an option. Dongles with a
   CSR8510 or Broadcom BCM20702 chip usually allow it at runtime: `btmgmt --index <n> public-addr 00:13:7B:xx:xx:xx`
   (kernel support per chip), or the `bdaddr` tool. Use a second adapter for this, so the PC's own address and its
   pairings stay as they are, and an address that no real controller has. An ESP32 (classic Bluetooth, address freely
   settable, SPP, USB serial to the PC) would also do, as a tiny relay that isn't a TX.
2. **Learn the message protocol on the wire.** With the PC holding a `00:13:7B` address: listen on RFCOMM channel 1
   on the PC, let a TX in online mode connect to the PC (telecode 12) and send messages (telecode 14). The bytes that
   arrive are the TX-to-TX protocol: framing, any handshake, and how the 16-byte messages are packed. Then the other
   direction: the model listens for the PC's address, the PC connects and replays it.
3. **`DirectConnection`** with the same interface as `RelayConnection` (relay protocol frames, now sent as TX-to-TX
   messages by the PC itself), and a `DirectLink` for the XCP bridge (`xcp/src/TxIo.hpp`).

Open: whether the check is only the address prefix (also the device class or name?), and whether a connection from
the PC is then treated as a message link or as online mode.
