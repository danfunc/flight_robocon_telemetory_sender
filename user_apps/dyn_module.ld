SECTIONS {
    . = 0x0;
    .text : {
        KEEP(*(.text.entry))
        *(.text*)
        *(.rodata*)
        *(.data*)
        *(.bss*)
    }
}
