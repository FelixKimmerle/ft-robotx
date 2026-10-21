/*
 * Linker script for ROBO TX Controller programs (download mode).
 * Based on ld.lcf from fischertechnik's C compiler package V1.2.
 *
 * The firmware copies the program file (code, constants and initialized data, in this order) to the start of the
 * program memory and calls the entry point named in the header at its very beginning (section .intro).
 */

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
    program (rwx) : ORIGIN = 0x30700000, LENGTH = 832K
}

SECTIONS
{
    .code :
    {
        KEEP(*(.intro))             /* program header, must come first */
        *(.text .text.*)
        *(.glue_7t .glue_7 .vfp11_veneer .v4_bx)
        . = ALIGN(4);
    } > program

    .rodata :
    {
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > program

    .data :
    {
        *(.data .data.*)
        . = ALIGN(4);
    } > program

    /* Not part of the program file; cleared by the dispatcher before PrgInit(). */
    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > program

    /* No exception unwinding on the controller. */
    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.comment)
        *(.note*)
    }
}
