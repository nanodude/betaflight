/*
*****************************************************************************
**
**  File        : stm32_flash_f446_drbl.ld
**
**  Abstract    : Linker script for STM32F446 Device with
**                512KByte FLASH, 128KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x08000000 to 0x08080000 512kb full flash,
0x08000000 to 0x08010000 64kb dRonin BL / empty
0x08010000 to 0x08020000 64kb settings
0x08020000 to 0x08060000 384kb firmware
*/


/* Specify the memory areas */
MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08020000, LENGTH = 384K
	RAM (rwx)       : ORIGIN = 0x20000000, LENGTH = 128K
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash_drbl.ld"
