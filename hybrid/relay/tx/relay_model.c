//=============================================================================
// Model TX for the relay protocol (../protocol/relay_protocol.h): accepts the relay's Bluetooth connection, carries
// out the PC's commands on this controller and its extensions, and sends back the values the PC subscribed to.
//
// Frames arrive in the Bluetooth receive callback and are queued; PrgTic() (every millisecond) carries them out,
// because it owns the transfer areas. Frames to the PC go out one at a time: the next one after the firmware
// confirmed the previous one (or after a timeout, since a confirmation can get lost).
//
// The relay's address is set at build time (FT_RELAY_ADDRESS in CMakeLists.txt).
//=============================================================================

#include "ROBO_TX_PRG.h"
#include "relay_protocol.h"

#define KEEP_RUNNING        0x7FFF  // PrgTic() result: call again in 1 ms
#define RX_QUEUE            8       // frames from the PC
#define TX_QUEUE            16      // frames to the PC
#define SEND_TIMEOUT_MS     50      // a lost send confirmation doesn't block sending
#define MIN_INTERVAL_MS     10      // the firmware updates the inputs every 10 ms
#define FULL_REPORT_MS      1000    // all subscribed values at least this often (heals lost frames)

typedef struct
{
    UINT8 length;
    UINT8 data[RELAY_FRAME_MAX];
} Frame;

typedef struct
{
    UINT32 mask;            // subscribed signals, 0 = none
    UINT16 interval;
    UINT32 last_sent;       // ms
    UINT32 last_full;       // ms
    BOOL32 have_values;
    UINT16 values[RELAY_SIG_COUNT];
} Subscription;

static UCHAR8 relay_address[BT_ADDR_LEN] = {RELAY_ADDRESS};

static TA * p_tas;
static int n_tas;

// Bluetooth state, written by the callbacks
static volatile BOOL32 connected;
static volatile BOOL32 start_receive;
static volatile BOOL32 connection_lost;
static volatile BOOL32 send_busy;

static Frame rx_queue[RX_QUEUE];
static volatile UINT32 rx_head;    // written by the receive callback
static UINT32 rx_tail;             // read by PrgTic

static Frame tx_queue[TX_QUEUE];
static UINT32 tx_head;
static UINT32 tx_tail;
static UINT32 send_started;        // ms

static UINT32 last_rx_time;        // ms
static BOOL32 watchdog_fired;
static BOOL32 have_last_seq;
static UINT8 last_seq;
static UINT8 last_status;
static UCHAR8 values_seq;

static Subscription subscriptions[TA_COUNT];
static char text[100];

static UINT32 Now(void)
{
    return p_tas[TA_LOCAL].hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
}

static void Show(const char * message)
{
    p_tas[TA_LOCAL].hook_table.DisplayMsg(&p_tas[TA_LOCAL], NULL);
    p_tas[TA_LOCAL].hook_table.DisplayMsg(&p_tas[TA_LOCAL], (char *)message);
}

//-----------------------------------------------------------------------------
// Bluetooth
//-----------------------------------------------------------------------------

static void ListenCallback(TA * p_ta_array, BT_CB * p_data)
{
    (void)p_ta_array;
    if (p_data->status == BT_CON_INDICATION)
    {
        connected = TRUE;
        start_receive = TRUE;
    }
    else if (p_data->status == BT_DISCON_INDICATION)
    {
        connected = FALSE;
        connection_lost = TRUE;
    }
}

static void SendCallback(TA * p_ta_array, BT_CB * p_data)
{
    (void)p_ta_array;
    (void)p_data;
    send_busy = FALSE;
}

static void ReceiveCallback(TA * p_ta_array, BT_RECV_CB * p_data)
{
    (void)p_ta_array;
    if (p_data->status != BT_MSG_INDICATION || p_data->msg_len < 2)
    {
        return;
    }
    UINT32 next = (rx_head + 1) % RX_QUEUE;
    if (next == rx_tail)
    {
        return; // full; the PC sends the command again when its ACK doesn't come
    }
    Frame * frame = &rx_queue[rx_head];
    frame->length = (UINT8)(p_data->msg_len <= RELAY_FRAME_MAX ? p_data->msg_len : RELAY_FRAME_MAX);
    p_tas[TA_LOCAL].hook_table.memcpy(frame->data, p_data->msg, frame->length);
    rx_head = next;
}

static UINT32 TxFree(void)
{
    return (tx_tail + TX_QUEUE - tx_head - 1) % TX_QUEUE;
}

static void Queue(const UINT8 * data, UINT32 length)
{
    if (TxFree() == 0)
    {
        return; // dropped; values are sent again, a lost ACK makes the PC repeat its command
    }
    Frame * frame = &tx_queue[tx_head];
    frame->length = (UINT8)length;
    p_tas[TA_LOCAL].hook_table.memcpy(frame->data, data, length);
    tx_head = (tx_head + 1) % TX_QUEUE;
}

static void SendNext(UINT32 now)
{
    if (send_busy && now - send_started < SEND_TIMEOUT_MS)
    {
        return;
    }
    if (tx_tail == tx_head || !connected)
    {
        send_busy = FALSE;
        return;
    }
    Frame * frame = &tx_queue[tx_tail];
    tx_tail = (tx_tail + 1) % TX_QUEUE;
    send_busy = TRUE;
    send_started = now;
    p_tas[TA_LOCAL].hook_table.BtSend(RELAY_CHANNEL, frame->length, frame->data, SendCallback);
}

//-----------------------------------------------------------------------------
// Motors, as Extension in the online library (online/cpp/src/Extension.cpp)
//-----------------------------------------------------------------------------

static INT16 GetI16(const UINT8 * p)
{
    return (INT16)(p[0] | p[1] << 8);
}

static UINT16 GetU16(const UINT8 * p)
{
    return (UINT16)(p[0] | p[1] << 8);
}

static BOOL32 MotorDone(TA * p_ta, UINT32 motor)
{
    // The firmware confirms a motor command by echoing its id.
    return p_ta->input.motor_pos_reached[motor] == p_ta->output.motor_ex_cmd_id[motor];
}

// The motor synchronized with this one (as master or slave), or -1.
static int SyncPartner(TA * p_ta, UINT32 motor)
{
    for (UINT32 other = 0; other < N_MOTOR; other++)
    {
        if (other != motor &&
            (p_ta->output.master[other] == motor + 1 || p_ta->output.master[motor] == other + 1))
        {
            return (int)other;
        }
    }
    return -1;
}

static void HardStop(TA * p_ta, UINT32 motor)
{
    p_ta->output.duty[2 * motor] = 0;
    p_ta->output.duty[2 * motor + 1] = 0;
    p_ta->output.distance[motor] = 0;
    p_ta->output.master[motor] = 0;
    p_ta->output.motor_ex_cmd_id[motor]++;
}

// Ends the distance/synchronization command of a motor and stops the motors synchronized with it.
static void ReleaseMotor(TA * p_ta, UINT32 motor)
{
    BOOL32 command_running = p_ta->output.distance[motor] != 0 || p_ta->output.master[motor] != 0;
    int partner;
    while ((partner = SyncPartner(p_ta, motor)) >= 0)
    {
        HardStop(p_ta, (UINT32)partner); // the firmware would hold it at our counter otherwise
        if (p_ta->output.master[motor] == partner + 1)
        {
            p_ta->output.master[motor] = 0;
        }
        command_running = TRUE;
    }
    if (command_running)
    {
        p_ta->output.distance[motor] = 0;
        p_ta->output.master[motor] = 0;
        p_ta->output.motor_ex_cmd_id[motor]++; // new settings only take effect with a new command
    }
}

static void SetMotor(TA * p_ta, UINT32 motor, INT16 duty)
{
    ReleaseMotor(p_ta, motor);
    p_ta->output.duty[2 * motor] = duty > 0 ? duty : 0;
    p_ta->output.duty[2 * motor + 1] = duty > 0 ? 0 : (INT16)-duty;
}

static void StopMotor(TA * p_ta, UINT32 motor)
{
    // A motor that finished its distance command stands already; a new command would only reset the counters.
    int partner = SyncPartner(p_ta, motor);
    BOOL32 finished = p_ta->output.distance[motor] != 0 && MotorDone(p_ta, motor);
    if (partner >= 0)
    {
        finished = finished && p_ta->output.distance[partner] != 0 && MotorDone(p_ta, (UINT32)partner);
    }
    if (finished)
    {
        p_ta->output.duty[2 * motor] = 0;
        p_ta->output.duty[2 * motor + 1] = 0;
        return;
    }
    SetMotor(p_ta, motor, 0);
}

static void StopAll(void)
{
    for (int i = 0; i < n_tas; i++)
    {
        for (UINT32 m = 0; m < N_MOTOR; m++)
        {
            HardStop(&p_tas[i], m);
        }
    }
}

//-----------------------------------------------------------------------------
// Signals
//-----------------------------------------------------------------------------

static UINT16 Signal(TA * p_ta, UINT32 signal)
{
    if (signal < RELAY_SIG_C1)
    {
        return (UINT16)p_ta->input.uni[signal];
    }
    if (signal < RELAY_SIG_C1_LEVEL)
    {
        return (UINT16)p_ta->input.counter[signal - RELAY_SIG_C1];
    }
    if (signal < RELAY_SIG_BUTTON_LEFT)
    {
        return (UINT16)p_ta->input.cnt_in[signal - RELAY_SIG_C1_LEVEL];
    }
    if (signal == RELAY_SIG_BUTTON_LEFT)
    {
        return (UINT16)p_ta->input.display_button_left;
    }
    if (signal == RELAY_SIG_BUTTON_RIGHT)
    {
        return (UINT16)p_ta->input.display_button_right;
    }
    return MotorDone(p_ta, signal - RELAY_SIG_M1_DONE) ? 1 : 0;
}

// Sends the changed subscribed values (all of them once a second) in frames of up to RELAY_MAX_VALUES.
static void Report(UINT32 ctrl, UINT32 now)
{
    Subscription * sub = &subscriptions[ctrl];
    if (sub->mask == 0 || now - sub->last_sent < sub->interval)
    {
        return;
    }
    TA * p_ta = &p_tas[ctrl];
    BOOL32 full = !sub->have_values || now - sub->last_full >= FULL_REPORT_MS;
    UINT32 changed = 0;
    UINT16 values[RELAY_SIG_COUNT];
    for (UINT32 s = 0; s < RELAY_SIG_COUNT; s++)
    {
        values[s] = Signal(p_ta, s);
        if ((sub->mask >> s & 1) && (full || values[s] != sub->values[s]))
        {
            changed |= 1u << s;
        }
    }
    if (changed == 0)
    {
        return;
    }
    if (TxFree() < 3) // keep room for ACKs; try again next tic
    {
        return;
    }

    for (UINT32 s = 0; s < RELAY_SIG_COUNT && TxFree() > 1; s++)
    {
        if (!(changed >> s & 1))
        {
            continue;
        }
        UINT32 count = RELAY_SIG_COUNT - s < RELAY_MAX_VALUES ? RELAY_SIG_COUNT - s : RELAY_MAX_VALUES;
        UINT8 frame[RELAY_FRAME_MAX];
        frame[0] = RELAY_VALUES;
        frame[1] = values_seq++;
        frame[2] = (UINT8)ctrl;
        frame[3] = (UINT8)s;
        frame[4] = (UINT8)count;
        for (UINT32 i = 0; i < count; i++)
        {
            frame[5 + 2 * i] = (UINT8)values[s + i];
            frame[6 + 2 * i] = (UINT8)(values[s + i] >> 8);
            sub->values[s + i] = values[s + i];
        }
        Queue(frame, 5 + 2 * count);
        s += count - 1;
    }
    sub->have_values = TRUE;
    sub->last_sent = now;
    if (full)
    {
        sub->last_full = now;
    }
}

//-----------------------------------------------------------------------------
// Commands
//-----------------------------------------------------------------------------

// Carries out one command; returns its status and fills `data` (up to 13 bytes) for the ACK.
static UINT8 Execute(const Frame * frame, UINT8 * data, UINT32 * data_length)
{
    const UINT8 * a = &frame->data[2]; // arguments
    UINT32 n = frame->length - 2u;
    UINT8 op = frame->data[0];
    *data_length = 0;

    if (op == RELAY_HELLO)
    {
        UINT8 extensions = 0;
        for (int i = 0; i < N_EXT; i++)
        {
            if (p_tas[TA_LOCAL].state.ext_dev_connect_state[i] == EXT_DEV_ONLINE)
            {
                extensions |= (UINT8)(1u << i);
            }
        }
        data[0] = RELAY_PROTOCOL_VERSION;
        data[1] = (UINT8)n_tas;
        data[2] = extensions;
        *data_length = 3;
        return RELAY_OK;
    }
    if (op == RELAY_PING)
    {
        return RELAY_OK;
    }
    if (op == RELAY_STOP_ALL)
    {
        StopAll();
        return RELAY_OK;
    }
    if (op == RELAY_TEXT)
    {
        if (n < 2 || a[0] >= sizeof(text) - 1)
        {
            return RELAY_BAD_ARGUMENT;
        }
        UINT32 offset = a[0];
        UINT32 count = n - 2;
        if (offset + count > sizeof(text) - 1)
        {
            count = sizeof(text) - 1 - offset;
        }
        p_tas[TA_LOCAL].hook_table.memcpy(&text[offset], &a[2], count);
        if (a[1])
        {
            text[offset + count] = 0;
            Show(text);
        }
        return RELAY_OK;
    }

    // Everything else addresses a controller and an index.
    if (n < 2 || a[0] >= n_tas)
    {
        return RELAY_BAD_ARGUMENT;
    }
    UINT32 ctrl = a[0];
    TA * p_ta = &p_tas[ctrl];
    UINT32 index = a[1];

    switch (op)
    {
    case RELAY_SET_PWM:
        if (n < 4 || index >= N_PWM_CHAN)
        {
            return RELAY_BAD_ARGUMENT;
        }
        p_ta->output.duty[index] = GetI16(&a[2]);
        return RELAY_OK;
    case RELAY_SET_MOTOR:
        if (n < 4 || index >= N_MOTOR)
        {
            return RELAY_BAD_ARGUMENT;
        }
        SetMotor(p_ta, index, GetI16(&a[2]));
        return RELAY_OK;
    case RELAY_MOTOR_DISTANCE:
        if (n < 6 || index >= N_MOTOR)
        {
            return RELAY_BAD_ARGUMENT;
        }
        SetMotor(p_ta, index, GetI16(&a[2]));
        p_ta->output.distance[index] = GetU16(&a[4]);
        p_ta->output.motor_ex_cmd_id[index]++;
        return RELAY_OK;
    case RELAY_MOTOR_SYNC:
    {
        if (n < 8 || index >= N_MOTOR || a[2] >= N_MOTOR || a[2] == index)
        {
            return RELAY_BAD_ARGUMENT;
        }
        UINT32 slave = a[2];
        INT16 duty = GetI16(&a[3]);
        UINT16 distance = GetU16(&a[5]);
        SetMotor(p_ta, index, duty);
        SetMotor(p_ta, slave, a[7] ? (INT16)-duty : duty);
        // The slave's entry names the master; both get the same distance (otherwise the pair stalls).
        p_ta->output.master[slave] = (UINT8)(index + 1);
        p_ta->output.distance[index] = distance;
        p_ta->output.distance[slave] = distance;
        p_ta->output.motor_ex_cmd_id[index]++;
        p_ta->output.motor_ex_cmd_id[slave]++;
        return RELAY_OK;
    }
    case RELAY_STOP_MOTOR:
        if (index >= N_MOTOR)
        {
            return RELAY_BAD_ARGUMENT;
        }
        StopMotor(p_ta, index);
        return RELAY_OK;
    case RELAY_RESET_COUNTER:
        if (index >= N_CNT)
        {
            return RELAY_BAD_ARGUMENT;
        }
        p_ta->output.cnt_reset_cmd_id[index]++;
        return RELAY_OK;
    case RELAY_INPUT_MODE:
        if (n < 4 || index >= N_UNI || (a[2] != MODE_U && a[2] != MODE_R && a[2] != MODE_ULTRASONIC))
        {
            return RELAY_BAD_ARGUMENT;
        }
        p_ta->config.uni[index].mode = a[2];
        p_ta->config.uni[index].digital = a[2] != MODE_ULTRASONIC && a[3] ? TRUE : FALSE;
        p_ta->state.config_id++;
        return RELAY_OK;
    case RELAY_MOTOR_MODE:
        if (n < 3 || index >= N_MOTOR)
        {
            return RELAY_BAD_ARGUMENT;
        }
        SetMotor(p_ta, index, 0); // switching ends a command; a partner motor stops
        p_ta->output.duty[2 * index] = 0;
        p_ta->output.duty[2 * index + 1] = 0;
        p_ta->config.motor[index] = a[2] ? TRUE : FALSE;
        p_ta->state.config_id++;
        return RELAY_OK;
    case RELAY_COUNTER_MODE:
        if (n < 3 || index >= N_CNT)
        {
            return RELAY_BAD_ARGUMENT;
        }
        p_ta->config.cnt[index].mode = a[2] ? 1 : 0;
        p_ta->state.config_id++;
        return RELAY_OK;
    case RELAY_READ:
    {
        if (index >= RELAY_SIG_COUNT)
        {
            return RELAY_BAD_ARGUMENT;
        }
        UINT16 value = Signal(p_ta, index);
        data[0] = (UINT8)value;
        data[1] = (UINT8)(value >> 8);
        *data_length = 2;
        return RELAY_OK;
    }
    case RELAY_SUBSCRIBE:
    {
        if (n < 7)
        {
            return RELAY_BAD_ARGUMENT;
        }
        // a[1..4]: mask (the index byte is the mask's lowest byte), a[5..6]: interval
        Subscription * sub = &subscriptions[ctrl];
        sub->mask = (UINT32)(a[1] | a[2] << 8 | a[3] << 16 | (UINT32)a[4] << 24) & ((1u << RELAY_SIG_COUNT) - 1);
        sub->interval = GetU16(&a[5]);
        if (sub->interval < MIN_INTERVAL_MS)
        {
            sub->interval = MIN_INTERVAL_MS;
        }
        sub->have_values = FALSE; // the next report sends everything
        sub->last_sent = 0;
        return RELAY_OK;
    }
    default:
        return RELAY_UNKNOWN_OP;
    }
}

static void Handle(const Frame * frame)
{
    UINT8 op = frame->data[0];
    UINT8 seq = frame->data[1];
    UINT8 reply[RELAY_FRAME_MAX];
    UINT32 data_length = 0;
    UINT8 status;

    // A repeated command (its ACK got lost) is only answered again: running a distance or a counter reset twice
    // would start it anew.
    if (have_last_seq && seq == last_seq && op != RELAY_READ && op != RELAY_HELLO)
    {
        status = last_status;
    }
    else
    {
        status = Execute(frame, &reply[3], &data_length);
        have_last_seq = TRUE;
        last_seq = seq;
        last_status = status;
    }
    reply[0] = RELAY_ACK;
    reply[1] = seq;
    reply[2] = status;
    Queue(reply, 3 + data_length);
}

//-----------------------------------------------------------------------------
// Program
//-----------------------------------------------------------------------------

void PrgInit(TA * p_ta_array, int ta_count)
{
    p_tas = p_ta_array;
    n_tas = ta_count;
    connected = FALSE;
    start_receive = FALSE;
    connection_lost = FALSE;
    send_busy = FALSE;
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    watchdog_fired = TRUE; // nothing to stop yet
    have_last_seq = FALSE;
    values_seq = 0;
    p_tas[TA_LOCAL].hook_table.memset(subscriptions, 0, sizeof(subscriptions));
    p_tas[TA_LOCAL].hook_table.BtStartListen(RELAY_CHANNEL, relay_address, ListenCallback);
    Show("Relay model\nwaiting for the relay");
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)p_ta_array;
    (void)ta_count;
    UINT32 now = Now();

    if (start_receive)
    {
        start_receive = FALSE;
        last_rx_time = now;
        p_tas[TA_LOCAL].hook_table.BtStartReceive(RELAY_CHANNEL, ReceiveCallback);
        Show("Relay model\nconnected");
    }
    if (connection_lost)
    {
        connection_lost = FALSE;
        StopAll();
        p_tas[TA_LOCAL].hook_table.memset(subscriptions, 0, sizeof(subscriptions));
        tx_head = tx_tail = 0;
        have_last_seq = FALSE;
        p_tas[TA_LOCAL].hook_table.BtStartListen(RELAY_CHANNEL, relay_address, ListenCallback);
        Show("Relay model\nconnection lost");
    }

    while (rx_tail != rx_head)
    {
        Handle(&rx_queue[rx_tail]);
        rx_tail = (rx_tail + 1) % RX_QUEUE;
        last_rx_time = now;
        watchdog_fired = FALSE;
    }

    // Watchdog: the PC went silent.
    if (!watchdog_fired && now - last_rx_time > RELAY_WATCHDOG_MS)
    {
        watchdog_fired = TRUE;
        StopAll(); // subscriptions stay: values keep coming when the PC is back
        Show("Relay model\nwatchdog: outputs off");
    }

    if (connected)
    {
        for (int i = 0; i < n_tas; i++)
        {
            Report((UINT32)i, now);
        }
    }
    SendNext(now);

    return KEEP_RUNNING;
}
