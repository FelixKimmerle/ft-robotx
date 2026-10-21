//=============================================================================
// Extension test, running on the master ROBO TX Controller (download mode).
// Master and extension 1 each have a button on I1 and a lamp on O1, wired crosswise:
// the master's button switches the extension's lamp and the other way round.
//
// The extension needs no program of its own: the master's program controls it through its transfer area
// (TA_EXT_1), just like a PC does in online mode. Set the extension's role to "Extension 1" in its menu.
//
// Same behavior as the online examples online/cpp/examples/extension_test.cpp and online/cs/examples/ExtensionTest.
//=============================================================================

#include "ROBO_TX_PRG.h"

#define BUTTON          0       // I1
#define LAMP            0       // O1
#define KEEP_RUNNING    0x7FFF  // PrgTic() result: call again in 1 ms

static BOOL8 extension_was_online;

static void configure(TA * p_ta)
{
    // A button is read as a digital resistance input (0 = open, 1 = closed).
    p_ta->config.uni[BUTTON].mode = MODE_R;
    p_ta->config.uni[BUTTON].digital = TRUE;
    // O1 and O2 as separate outputs, not as motor M1.
    p_ta->config.motor[LAMP / 2] = FALSE;
    // Tell the firmware the configuration changed.
    p_ta->state.config_id += 1;
}

void PrgInit(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    configure(&p_ta_array[TA_LOCAL]);
    configure(&p_ta_array[TA_EXT_1]);
    extension_was_online = TRUE; // so a missing extension is reported right away
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_master = &p_ta_array[TA_LOCAL];
    TA * p_extension = &p_ta_array[TA_EXT_1];

    // Crosswise: each button switches the other controller's lamp.
    p_extension->output.duty[LAMP] = p_master->input.uni[BUTTON] ? DUTY_MAX : 0;
    p_master->output.duty[LAMP] = p_extension->input.uni[BUTTON] ? DUTY_MAX : 0;

    // Say so on the master's display if extension 1 isn't there.
    BOOL8 online = p_master->state.ext_dev_connect_state[TA_EXT_1 - 1] == EXT_DEV_ONLINE;
    if (online != extension_was_online)
    {
        p_master->hook_table.DisplayMsg(p_master, online ? NULL : "Extension 1 not connected");
        extension_was_online = online;
    }

    return KEEP_RUNNING;
}
