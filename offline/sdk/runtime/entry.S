// Entry point called by the ROBO TX Controller firmware every millisecond:
//     int robotx_entry(TA * p_ta_array, int ta_count)
//
// The firmware was built for the old APCS ABI, which only keeps the stack 4-byte aligned. Code built for today's
// AAPCS may rely on 8-byte alignment (e.g. LDRD/STRD on the stack, which must be 8-byte aligned on ARMv5TE), so
// align the stack here and restore the firmware's stack pointer afterwards.

    .syntax unified
    .arm

    .section .text.robotx_entry, "ax", %progbits
    .global robotx_entry
    .type robotx_entry, %function
robotx_entry:
    push    {r4, lr}
    mov     r4, sp
    bic     sp, sp, #7
    bl      robotx_dispatch     // arguments r0, r1 and the result r0 pass through unchanged
    mov     sp, r4
    pop     {r4, pc}
    .size robotx_entry, . - robotx_entry
