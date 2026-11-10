//=============================================================================
// Bluetooth echo for the model TX: accepts a connection from the relay TX on channel 1 and sends every message
// back. With ftrobotx_bt_ping -a <this controller's address> on the PC, this measures messages between two
// controllers without a PC link on this one.
//
// The relay's address is set at build time (FT_RELAY_ADDRESS in CMakeLists.txt).
//=============================================================================

#include "ROBO_TX_PRG.h"

#define CHANNEL         1
#define KEEP_RUNNING    0x7FFF  // PrgTic() result: call again in 1 ms

static UCHAR8 relay_address[BT_ADDR_LEN] = {RELAY_ADDRESS};

// Written by the Bluetooth callbacks, read by PrgTic().
static volatile BOOL32 connected;
static volatile BOOL32 start_receive;   // incoming connection: start receiving in the next tic
static volatile BOOL32 listen_again;    // connection gone: listen for the next one
static volatile BOOL32 display_changed;
static volatile UINT32 messages;
static volatile UINT32 send_errors;
static volatile UINT16 last_status;

static TA * p_master;

static void ListenCallback(TA * p_ta_array, BT_CB * p_data)
{
    (void)p_ta_array;
    last_status = p_data->status;
    if (p_data->status == BT_CON_INDICATION)
    {
        connected = TRUE;
        start_receive = TRUE;
    }
    else if (p_data->status == BT_DISCON_INDICATION)
    {
        connected = FALSE;
        listen_again = TRUE;
    }
    display_changed = TRUE;
}

static void SendCallback(TA * p_ta_array, BT_CB * p_data)
{
    (void)p_ta_array;
    if (p_data->status != BT_SUCCESS)
    {
        send_errors++;
        last_status = p_data->status;
    }
}

static void ReceiveCallback(TA * p_ta_array, BT_RECV_CB * p_data)
{
    (void)p_ta_array;
    if (p_data->status == BT_MSG_INDICATION)
    {
        messages++;
        UINT32 length = p_data->msg_len <= BT_MSG_LEN ? p_data->msg_len : BT_MSG_LEN;
        p_master->hook_table.BtSend(CHANNEL, length, p_data->msg, SendCallback);
    }
    else
    {
        last_status = p_data->status;
        display_changed = TRUE;
    }
}

void PrgInit(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    p_master = &p_ta_array[TA_LOCAL];
    connected = FALSE;
    start_receive = FALSE;
    listen_again = FALSE;
    messages = 0;
    send_errors = 0;
    last_status = BT_SUCCESS;
    display_changed = TRUE;
    p_master->hook_table.BtStartListen(CHANNEL, relay_address, ListenCallback);
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)p_ta_array;
    (void)ta_count;

    if (start_receive)
    {
        start_receive = FALSE;
        p_master->hook_table.BtStartReceive(CHANNEL, ReceiveCallback);
    }
    if (listen_again)
    {
        listen_again = FALSE;
        p_master->hook_table.BtStartListen(CHANNEL, relay_address, ListenCallback);
    }

    // State on the display, when it changes (messages are counted, but not shown on every one).
    static UINT32 shown_messages = (UINT32)-1;
    static UINT32 next_refresh;
    UINT32 now = p_master->hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
    if ((display_changed || shown_messages != messages) && (INT32)(now - next_refresh) >= 0)
    {
        char text[64];
        p_master->hook_table.sprintf(text, "BtEcho %s\nmessages %u\nsend errors %u\nstatus %u",
                                     connected ? "connected" : "listening", (unsigned)messages,
                                     (unsigned)send_errors, (unsigned)last_status);
        p_master->hook_table.DisplayMsg(p_master, NULL);
        p_master->hook_table.DisplayMsg(p_master, text);
        display_changed = FALSE;
        shown_messages = messages;
        next_refresh = now + 500;
    }

    return KEEP_RUNNING;
}
