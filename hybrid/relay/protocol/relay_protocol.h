// Relay protocol: commands from the PC to the model TX and values back, one Bluetooth message (frame) each, relayed by
// a TX in online mode. Shared by the program on the model TX (C) and the PC side (C++); see ../README.md.
//
// Every frame: [op][seq][arguments...], at most RELAY_FRAME_MAX bytes, numbers little endian. The commands are
// written as instructions (opcode + fixed arguments), so they can become instructions of a VM with control flow later.
#ifndef RELAY_PROTOCOL_H
#define RELAY_PROTOCOL_H

#define RELAY_PROTOCOL_VERSION  1
#define RELAY_FRAME_MAX         16      // the receiving firmware keeps only 16 bytes of a message
#define RELAY_CHANNEL           1       // Bluetooth channel on both controllers
#define RELAY_WATCHDOG_MS       1000    // model: no frame from the PC this long -> all outputs off
#define RELAY_MAX_VALUES        5       // values per RELAY_VALUES frame

// Opcodes. Commands (PC -> model) are each answered with RELAY_ACK (same seq). "ctrl" is the controller on the
// model's side: 0 = the model TX itself, 1 ... 8 = its extensions (EXT cable).
enum RelayOp
{
    // [version] -> ACK data: protocol version, number of controllers (1 + extensions), extensions online (bit mask)
    RELAY_HELLO = 0x01,
    // [] -> ACK; keeps the watchdog quiet
    RELAY_PING = 0x02,

    // Outputs
    RELAY_SET_PWM = 0x10,           // ctrl, output 0..7, duty (i16, 0..512)
    RELAY_SET_MOTOR = 0x11,         // ctrl, motor 0..3, duty (i16, -512..512), runs endlessly, ends a distance/sync
    RELAY_MOTOR_DISTANCE = 0x12,    // ctrl, motor, duty (i16), distance (u16 counter pulses)
    RELAY_MOTOR_SYNC = 0x13,        // ctrl, master, slave, duty (i16), distance (u16, 0 = endless), opposite (0/1)
    RELAY_STOP_MOTOR = 0x14,        // ctrl, motor; also stops a motor synchronized with it
    RELAY_STOP_ALL = 0x15,          // all motors and outputs of all controllers
    RELAY_RESET_COUNTER = 0x16,     // ctrl, counter 0..3

    // Configuration
    RELAY_INPUT_MODE = 0x20,        // ctrl, input 0..7, mode (0 voltage, 1 resistance, 3 ultrasonic), digital (0/1)
    RELAY_MOTOR_MODE = 0x21,        // ctrl, pair 0..3, motor (1) or two single outputs (0)
    RELAY_COUNTER_MODE = 0x22,      // ctrl, counter 0..3, inverted (0: counts 0 -> 1, 1: counts 1 -> 0)

    // Values
    RELAY_READ = 0x30,              // ctrl, signal -> ACK data: value (u16)
    RELAY_SUBSCRIBE = 0x31,         // ctrl, signals (u32 bit mask, 0 = stop), interval (u16 ms, minimum 10)

    // Display of the model TX
    RELAY_TEXT = 0x40,              // offset (0..99), last (0/1), up to 12 characters; the last piece shows the text

    // Model -> PC
    RELAY_ACK = 0x80,               // seq of the command, status, data (depends on the command)
    RELAY_VALUES = 0x81,            // seq (counts up, for loss detection), ctrl, first signal, count, values (u16)
};

// RELAY_ACK status
enum RelayStatus
{
    RELAY_OK = 0,
    RELAY_UNKNOWN_OP = 1,
    RELAY_BAD_ARGUMENT = 2,         // e.g. a controller that isn't connected, or motor == slave
};

// Signals of one controller, for RELAY_READ, RELAY_SUBSCRIBE and RELAY_VALUES
enum RelaySignal
{
    RELAY_SIG_I1 = 0,               // I1 ... I8: 0..7, value in the unit of the input's mode
    RELAY_SIG_C1 = 8,               // C1 ... C4 counter values: 8..11
    RELAY_SIG_C1_LEVEL = 12,        // C1 ... C4 input levels (0/1): 12..15
    RELAY_SIG_BUTTON_LEFT = 16,     // display buttons: time pressed in 10 ms, 0 = not pressed
    RELAY_SIG_BUTTON_RIGHT = 17,
    RELAY_SIG_M1_DONE = 18,         // M1 ... M4: 1 once the last distance command is done (or none runs): 18..21
    RELAY_SIG_COUNT = 22,
};

#endif // RELAY_PROTOCOL_H
