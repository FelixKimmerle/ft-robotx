//=============================================================================
// Header file with definition of the software interface to the
// ROBO TX Controller program.
// Can be used for building C-programs which can run under control
// of the ROBO TX Controller firmware in download (local) mode.
//
// Disclaimer - Exclusion of Liability
//
// This software is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. It can be used and modified by anyone
// free of any license obligations or authoring rights.
//
// From fischertechnik's C compiler package "C_Compiler_RoboTXC" V1.2 (firmware 1.30).
// Changed for ft-robotx (modern arm-none-eabi-gcc instead of YAGARTO GCC 4.4.1):
//   - no #include <math.h> (would need newlib; include it yourself if you need it)
//   - NULL only defined if not defined yet
//   - no bool enum in C23, where bool is a keyword
//=============================================================================

#ifndef __ROBO_TX_PRG_H__
#define __ROBO_TX_PRG_H__


#define CALL_CYCLE_MS    1  // firmware calls program each millisecond

#define MIN(a, b) ((a <= b) ? a : b)
#define MAX(a, b) ((a >= b) ? a : b)

#define TA_VERSION          0x08010101  // Transfer Area version. Is used by firmware to check if format of the
                                        // Transfer Area used in program is the same as the one used in firmware.

#define PRG_MEM_START       0x30700000  // start address of the program memory
#define PRG_MEM_SIZE        0x000D0000  // size of the program memory

#define PRG_MAGIC           0x336699AA

#define MAX_FRAME_SIZE      1024  // maximum size of displayable frame (display buffer size)

#ifndef NULL
#if defined(__cplusplus)
    #define NULL            0L
#else
    #define NULL            ((void *)0)
#endif
#endif

#undef  TRUE
#define TRUE                (1 == 1)

#undef  FALSE
#define FALSE               (!TRUE)

#if !defined(__cplusplus) && (!defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L)
typedef enum
{
    false = FALSE,
    true  = TRUE
} bool;
#endif // !__cplusplus && < C23

typedef signed char         CHAR8;
typedef unsigned char       UCHAR8;

typedef unsigned char       BOOL8;      //  boolean variable (should be TRUE or FALSE)
typedef unsigned short      BOOL16;     //  boolean variable (should be TRUE or FALSE)
typedef unsigned long       BOOL32;     //  boolean variable (should be TRUE or FALSE)

typedef signed char         INT8;
typedef signed short        INT16;
typedef signed int          INT32;

typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

#include "ROBO_TX_FW.h"

enum bt_commands_e
{
    CMD_NO_CMD,
    CMD_CONNECT,
    CMD_DISCONNECT,
    CMD_START_LISTEN,
    CMD_STOP_LISTEN,
    CMD_START_RECEIVE,
    CMD_STOP_RECEIVE,
    CMD_SEND
};

extern UCHAR8 bt_address_table[BT_CNT_MAX][BT_ADDR_LEN];

// At the beginning of the program code should be this structure
struct prg_code_intro_s
{
    unsigned long           magic;      // PRG_MAGIC
    FT_VER                  ta_version; // TA_VERSION
    unsigned long           entry;      // entry point of the program, usually address of
                                        // the program dispatcher (PrgDisp) function
};


// This function is called only once by the program dispatcher (PrgDisp) at the very first call
// of the program dispatcher by the firmware
void PrgInit
(
    TA * p_ta,  // pointer to the array of transfer area parts
    int n_ta    // number of transfer area parts in array (equal to TA_COUNT)
);


// This function is called periodically by the program dispatcher (PrgDisp) with the period
// of CALL_CYCLE_MS
int PrgTic
(
    TA * p_ta,  // pointer to the array of transfer area parts
    int n_ta    // number of transfer area parts in array (equal to TA_COUNT)
);


// This function converts the status of the executed Bluetooth command to a string
// and displays this string on the display of a ROBO TX Controller
BOOL32 BtDisplayCommandStatus
(
    TA * p_ta,
    UCHAR8 * bt_address,
    UINT32 channel,
    enum bt_commands_e command,
    CHAR8 command_status
);


#endif // __ROBO_TX_PRG_H__
