//=============================================================================
// Display demo, running on the ROBO TX Controller itself (download mode): shows how to use the graphics library
// robotx_gfx.h. The display shows a title, the seconds since the start as a large number, a progress bar for the
// current second, and a box for the button on I1 that fills while it is pressed.
//
// While the program owns the display, the firmware's "Stop" label is hidden: press both display buttons to stop it
// (or use ftrobotx-load stop).
//=============================================================================

#include "ROBO_TX_PRG.h"
#include "robotx_gfx.h"

#define BUTTON          0       // I1
#define REFRESH_MS      100     // redraw ten times per second
#define KEEP_RUNNING    0x7FFF  // PrgTic() result: call again in 1 ms

static UINT32 start_time;
static UINT32 last_refresh;

void PrgInit(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    // A button is read as a digital resistance input (0 = open, 1 = closed).
    p_ta->config.uni[BUTTON].mode = MODE_R;
    p_ta->config.uni[BUTTON].digital = TRUE;
    p_ta->state.config_id += 1;

    start_time = p_ta->hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
    last_refresh = start_time - REFRESH_MS; // draw right away

    gfx_begin(p_ta);
}

static void draw(UINT32 elapsed_ms, BOOL8 pressed)
{
    gfx_clear();

    // Frame and title
    gfx_rect(0, 0, GFX_WIDTH, GFX_HEIGHT, GFX_BLACK);
    const char * title = "ft-robotx offline";
    gfx_text((GFX_WIDTH - gfx_text_width(title, 1)) / 2, 3, title, 1, GFX_BLACK);
    gfx_line(1, 12, GFX_WIDTH - 2, 12, GFX_BLACK);

    // Seconds since the start, large
    int x = gfx_number(6, 20, (long)(elapsed_ms / 1000), 3, GFX_BLACK);
    gfx_text(x + 2, 34, "s", 1, GFX_BLACK);

    // Progress through the current second
    int width = (int)((elapsed_ms % 1000) * 70 / 1000);
    gfx_rect(6, 50, 72, 8, GFX_BLACK);
    gfx_fill_rect(7, 51, width, 6, GFX_BLACK);

    // Button on I1: an empty box, filled while pressed
    gfx_text(95, 18, "I1", 1, GFX_BLACK);
    gfx_rect(90, 28, 24, 24, GFX_BLACK);
    if (pressed)
    {
        gfx_fill_rect(93, 31, 18, 18, GFX_BLACK);
    }
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    UINT32 now = p_ta->hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
    if (now - last_refresh >= REFRESH_MS)
    {
        last_refresh = now;
        draw(now - start_time, p_ta->input.uni[BUTTON] != 0);
        gfx_show(p_ta);
    }

    return KEEP_RUNNING;
}
