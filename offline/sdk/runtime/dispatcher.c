//=============================================================================
// Program header and dispatcher of a ROBO TX Controller program (download mode).
// Based on prg_disp.c from fischertechnik's C compiler package V1.2.
//
// Every program provides PrgInit() (called once) and PrgTic() (called every
// millisecond); see ROBO_TX_PRG.h.
//=============================================================================

#include <stddef.h>

#include "ROBO_TX_PRG.h"

// The firmware and the program share the transfer area, so its layout must be exactly the one the firmware was
// built with (YAGARTO GCC 4.4.1, 2012). The sizes are the ones documented in ROBO_TX_FW.h; if a compiler or flag
// change ever breaks them, the build fails here instead of the program misbehaving on the controller.
_Static_assert(sizeof(TA_INFO) == 64, "TA_INFO layout differs from the firmware");
_Static_assert(sizeof(TA_STATE) == 100, "TA_STATE layout differs from the firmware");
_Static_assert(sizeof(TA_CONFIG) == 88, "TA_CONFIG layout differs from the firmware");
_Static_assert(sizeof(TA_INPUT) == 68, "TA_INPUT layout differs from the firmware");
_Static_assert(sizeof(TA_OUTPUT) == 44, "TA_OUTPUT layout differs from the firmware");
_Static_assert(sizeof(TA_DISPLAY) == 108, "TA_DISPLAY layout differs from the firmware");
_Static_assert(sizeof(TA_STATUS) == 4, "TA_STATUS layout differs from the firmware");
_Static_assert(sizeof(TA_CHANGE) == 8, "TA_CHANGE layout differs from the firmware");
_Static_assert(sizeof(TA_TIMER) == 12, "TA_TIMER layout differs from the firmware");
// 35 function pointers; the "132 bytes" in ROBO_TX_FW.h is from before the two I2C functions were added.
_Static_assert(sizeof(TA_HOOK_TABLE) == 35 * 4, "TA_HOOK_TABLE layout differs from the firmware");
_Static_assert(offsetof(TA, hook_table) == 524, "hook table is not where the firmware puts it");
_Static_assert(sizeof(TA) == 1024, "a transfer area must be exactly 1024 bytes");

// Entry point in entry.S; it aligns the stack and calls robotx_dispatch().
int robotx_entry(TA * p_ta_array, int ta_count);

// The firmware recognizes a program by this header at the very start of the program memory.
__attribute__((section(".intro"), used)) const struct prg_code_intro_s prg_code_intro = {
    /* magic      */ PRG_MAGIC,
    /* ta_version */ {TA_VERSION},
    /* entry      */ (unsigned long)robotx_entry,
};

// From the linker script.
extern char __bss_start[];
extern char __bss_end[];

// Called by robotx_entry every millisecond while the program runs.
int robotx_dispatch(TA * p_ta_array, int ta_count)
{
    TA * p_ta = &p_ta_array[TA_LOCAL];

    if (!p_ta->state.pgm_initialized)
    {
        // The firmware only copies the program file into memory; zero-initialized variables are not part of the
        // file, so clear them before the program starts.
        p_ta->hook_table.memset(__bss_start, 0, (UINT32)(__bss_end - __bss_start));

        PrgInit(p_ta_array, ta_count);

        p_ta->state.pgm_initialized = TRUE;
    }

    return PrgTic(p_ta_array, ta_count);
}
