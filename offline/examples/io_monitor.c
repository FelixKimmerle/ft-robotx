//=============================================================================
// I/O monitor, running on the ROBO TX Controller itself (download mode): a minimal, display-only relative of the
// ftrobotx_gui test tool. Shows the inputs and counters of the master and all connected extensions on the display;
// the display buttons flip through the pages (left = back, right = next):
//
//   Info       name, firmware, connected extensions, runtime
//   Digital    I1 ... I8 as boxes, filled when the input is 1
//   Ohm        I1 ... I8 as resistance with a bar
//   mV         I1 ... I8 as voltage with a bar
//   Counters   C1 ... C4 with level and count
//
// The digital, ohm and mV pages exist for every controller and set its inputs to that mode while shown.
// To stop the program, press both display buttons (or use ftrobotx-load stop).
//=============================================================================

#include "ROBO_TX_PRG.h"
#include "robotx_gfx.h"

#define REFRESH_MS      100     // redraw ten times per second
#define KEEP_RUNNING    0x7FFF  // PrgTic() result: call again in 1 ms

enum PageKind
{
    PAGE_DIGITAL,
    PAGE_OHM,
    PAGE_MV,
    PAGE_COUNTERS,
    PAGE_KINDS,
};

static const char * const page_names[PAGE_KINDS] = {"Digital", "Ohm", "mV", "Counters"};

// Input mode each page sets: mode, digital
static const UINT8 page_modes[PAGE_KINDS][2] = {
    {MODE_R, TRUE},  // digital: buttons, switches, ...
    {MODE_R, FALSE}, // resistance in ohm
    {MODE_U, FALSE}, // voltage in mV
    {MODE_R, TRUE},  // counters: inputs unchanged from the digital page
};

static int page;           // 0 = info, then PAGE_KINDS pages per controller
static int configured[TA_COUNT]; // page kind each controller's inputs are set up for, -1 = none
static BOOL8 left_was_pressed;
static BOOL8 right_was_pressed;
static UINT32 start_time;
static UINT32 last_refresh;

// Controllers to show: the master and every connected extension (transfer area indices).
static int controllers(TA * p_ta_array, int list[TA_COUNT])
{
    int count = 0;
    list[count++] = TA_LOCAL;
    for (int i = 0; i < N_EXT; ++i)
    {
        if (p_ta_array[TA_LOCAL].state.ext_dev_connect_state[i] == EXT_DEV_ONLINE)
        {
            list[count++] = TA_EXT_1 + i;
        }
    }
    return count;
}

static void configure_inputs(TA * p_ta, int ta_index, int kind)
{
    if (kind == PAGE_COUNTERS || configured[ta_index] == kind)
    {
        return;
    }
    for (int i = 0; i < N_UNI; ++i)
    {
        p_ta->config.uni[i].mode = page_modes[kind][0];
        p_ta->config.uni[i].digital = page_modes[kind][1];
    }
    p_ta->state.config_id += 1;
    configured[ta_index] = kind;
}

static const char * const controller_names[TA_COUNT] = {"Master", "Ext1", "Ext2", "Ext3", "Ext4",
                                                         "Ext5",   "Ext6", "Ext7", "Ext8"};

// Writes "page/pages" into text (at least 6 characters).
static void format_page(char * text, int page_number, int page_count)
{
    int values[2] = {page_number, page_count};
    for (int i = 0; i < 2; ++i)
    {
        if (values[i] >= 10)
        {
            *text++ = (char)('0' + values[i] / 10);
        }
        *text++ = (char)('0' + values[i] % 10);
        if (i == 0)
        {
            *text++ = '/';
        }
    }
    *text = '\0';
}

// Black bar across the top: title on the left, "page/pages" on the right.
static void draw_header(const char * left, const char * middle, int page_number, int page_count)
{
    gfx_fill_rect(0, 0, GFX_WIDTH, 9, GFX_BLACK);
    int x = gfx_text(1, 1, left, 1, GFX_WHITE);
    gfx_text(x + GFX_CHAR_WIDTH, 1, middle, 1, GFX_WHITE);

    char pages[6];
    format_page(pages, page_number, page_count);
    gfx_text(GFX_WIDTH - 1 - gfx_text_width(pages, 1), 1, pages, 1, GFX_WHITE);
}

static void draw_info(TA * p_ta_array, const int * list, int count, UINT32 elapsed_ms)
{
    TA * p_master = &p_ta_array[TA_LOCAL];

    gfx_text(0, 13, p_master->info.device_name, 1, GFX_BLACK);

    int x = gfx_text(0, 23, "Firmware ", 1, GFX_BLACK);
    x = gfx_number(x, 23, p_master->info.version.firmware.part.c, 1, GFX_BLACK);
    x = gfx_text(x, 23, ".", 1, GFX_BLACK);
    UINT8 minor = p_master->info.version.firmware.part.d;
    if (minor < 10)
    {
        x = gfx_text(x, 23, "0", 1, GFX_BLACK);
    }
    gfx_number(x, 23, minor, 1, GFX_BLACK);

    x = gfx_text(0, 33, "Extensions ", 1, GFX_BLACK);
    if (count == 1)
    {
        gfx_text(x, 33, "none", 1, GFX_BLACK);
    }
    for (int i = 1; i < count; ++i)
    {
        x = gfx_number(x, 33, list[i], 1, GFX_BLACK) + GFX_CHAR_WIDTH;
    }

    x = gfx_text(0, 43, "Running ", 1, GFX_BLACK);
    x = gfx_number(x, 43, (long)(elapsed_ms / 1000), 1, GFX_BLACK);
    gfx_text(x, 43, " s", 1, GFX_BLACK);

    gfx_text(0, 56, "< > pages  << >> stop", 1, GFX_BLACK);
}

static void draw_digital(TA * p_ta)
{
    for (int i = 0; i < N_UNI; ++i)
    {
        int x = (i % 4) * 32 + 4;
        int y = i < 4 ? 12 : 38;
        char label[3] = {'I', (char)('1' + i), '\0'};
        gfx_text(x + 6, y, label, 1, GFX_BLACK);
        gfx_rect(x + 2, y + 9, 22, 14, GFX_BLACK);
        if (p_ta->input.uni[i])
        {
            gfx_fill_rect(x + 4, y + 11, 18, 10, GFX_BLACK);
        }
    }
}

// Two columns of four inputs: "I1 4999" with a bar below, full at `maximum`; at or above `overrun` shows "over".
static void draw_analog(TA * p_ta, long maximum, long overrun)
{
    for (int i = 0; i < N_UNI; ++i)
    {
        int x = i < 4 ? 0 : 66;
        int y = 12 + (i % 4) * 13;
        char label[4] = {'I', (char)('1' + i), ' ', '\0'};
        int value_x = gfx_text(x, y, label, 1, GFX_BLACK);
        long value = p_ta->input.uni[i];
        if (value >= overrun)
        {
            gfx_text(value_x, y, "over", 1, GFX_BLACK);
            value = maximum;
        }
        else
        {
            gfx_number(value_x, y, value, 1, GFX_BLACK);
        }
        long bar = (value > maximum ? maximum : value) * 60 / maximum;
        gfx_fill_rect(x, y + 10, (int)bar, 2, GFX_BLACK);
    }
}

static void draw_counters(TA * p_ta)
{
    for (int i = 0; i < N_CNT; ++i)
    {
        int y = 14 + i * 12;
        char label[3] = {'C', (char)('1' + i), '\0'};
        gfx_text(0, y, label, 1, GFX_BLACK);
        // level as a small box
        gfx_rect(18, y - 1, 9, 9, GFX_BLACK);
        if (p_ta->input.cnt_in[i])
        {
            gfx_fill_rect(20, y + 1, 5, 5, GFX_BLACK);
        }
        gfx_number(36, y, (UINT16)p_ta->input.counter[i], 1, GFX_BLACK);
    }
}

static void draw(TA * p_ta_array, UINT32 elapsed_ms)
{
    int list[TA_COUNT];
    int count = controllers(p_ta_array, list);
    int page_count = 1 + count * PAGE_KINDS;
    if (page >= page_count)
    {
        page = page_count - 1; // an extension went away
    }

    gfx_clear();
    if (page == 0)
    {
        draw_header("ROBO TX", "Info", 1, page_count);
        draw_info(p_ta_array, list, count, elapsed_ms);
        return;
    }

    int ta_index = list[(page - 1) / PAGE_KINDS];
    int kind = (page - 1) % PAGE_KINDS;
    TA * p_ta = &p_ta_array[ta_index];
    configure_inputs(p_ta, ta_index, kind);

    draw_header(controller_names[ta_index], page_names[kind], page + 1, page_count);

    switch (kind)
    {
    case PAGE_DIGITAL:
        draw_digital(p_ta);
        break;
    case PAGE_OHM:
        draw_analog(p_ta, 4999, 5000);
        break;
    case PAGE_MV:
        draw_analog(p_ta, 9999, 10000);
        break;
    default:
        draw_counters(p_ta);
        break;
    }
}

void PrgInit(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    page = 0;
    for (int i = 0; i < TA_COUNT; ++i)
    {
        configured[i] = -1;
    }
    left_was_pressed = FALSE;
    right_was_pressed = FALSE;
    start_time = p_ta->hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
    last_refresh = start_time - REFRESH_MS; // draw right away

    gfx_begin(p_ta);
}

int PrgTic(TA * p_ta_array, int ta_count)
{
    (void)ta_count;
    TA * p_ta = &p_ta_array[TA_LOCAL];

    // Display buttons: the input says for how many milliseconds a button has been held; act when it's pressed.
    BOOL8 left = p_ta->input.display_button_left > 0;
    BOOL8 right = p_ta->input.display_button_right > 0;
    BOOL8 page_changed = FALSE;
    if (left && !left_was_pressed && !right)
    {
        int list[TA_COUNT];
        int page_count = 1 + controllers(p_ta_array, list) * PAGE_KINDS;
        page = (page + page_count - 1) % page_count;
        page_changed = TRUE;
    }
    if (right && !right_was_pressed && !left)
    {
        int list[TA_COUNT];
        int page_count = 1 + controllers(p_ta_array, list) * PAGE_KINDS;
        page = (page + 1) % page_count;
        page_changed = TRUE;
    }
    left_was_pressed = left;
    right_was_pressed = right;

    UINT32 now = p_ta->hook_table.GetSystemTime(TIMER_UNIT_MILLISECONDS);
    if (page_changed || now - last_refresh >= REFRESH_MS)
    {
        last_refresh = now;
        draw(p_ta_array, now - start_time);
        gfx_show(p_ta);
    }

    return KEEP_RUNNING;
}
