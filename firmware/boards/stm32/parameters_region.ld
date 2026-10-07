/* Application reservation. Keep the original library linker script intact. */
__firmware_flash_end = LOADADDR(.data) + SIZEOF(.data);
__parameters_bank_a = 0x08040000;
__parameters_bank_b = 0x08060000;
ASSERT(__firmware_flash_end <= __parameters_bank_a,
       "Firmware exceeds 256 KiB and overlaps internal parameter Flash")
