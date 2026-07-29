/* Code placed in CCM SRAM, loaded from flash and copied over at startup */
SECTIONS
{
    .ccmram : ALIGN(4)
    {
        __sccmram = .;
        *(.ccmram .ccmram.*);
        . = ALIGN(4);
        __eccmram = .;
    } > CCMRAM AT > FLASH
    __siccmram = LOADADDR(.ccmram);
} INSERT BEFORE .data;
