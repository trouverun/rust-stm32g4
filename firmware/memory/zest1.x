MEMORY
{
    FLASH  : ORIGIN = 0x08004000, LENGTH = 236K /* BANK_1 (offset by 16K bootloader) */
    RAM    : ORIGIN = 0x20000000, LENGTH =  96K /* SRAM1 + SRAM2 */
    CCMRAM : ORIGIN = 0x10000000, LENGTH =  32K /* zero wait state code */
}