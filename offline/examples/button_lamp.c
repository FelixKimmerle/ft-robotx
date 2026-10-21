//=============================================================================
// Button and lamp, running on the ROBO TX Controller itself (download mode):
// the lamp on O1 lights while the button on I1 is pressed.
//
// Same behavior as the online examples online/cpp/examples/button_lamp.cpp and online/cs/examples/ButtonLamp.
//=============================================================================

#include "ROBO_TX_PRG.h"

#define BUTTON          0       // I1
#define LAMP            0       // O1
#define KEEP_RUNNING    0x7FFF  // PrgTic() result: call again in 1 ms

void PrgInit(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    // A button is read as a digital resistance input (0 = open, 1 = closed).
    p_ta->config.uni[BUTTON].mode = MODE_R;
    p_ta->config.uni[BUTTON].digital = TRUE;
    // O1 and O2 as separate outputs, not as motor M1.
    p_ta->config.motor[LAMP / 2] = FALSE;
    // Tell the firmware the configuration changed.
    p_ta->state.config_id += 1;
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    p_ta->output.duty[LAMP] = p_ta->input.uni[BUTTON] ? DUTY_MAX : 0;

    return KEEP_RUNNING;
}
