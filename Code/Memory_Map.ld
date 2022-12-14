/******************************************************************************************
  Filename    : Memory_Map.ld
  
  Core        : ARM Cortex®-M7
  
  MCU         : STM32F746ZGT6
    
  Author      : Chalandi Amine
 
  Owner       : Chalandi Amine
  
  Date        : 22.11.2022
  
  Description : Linker description file for STM32F746ZGT6
  
******************************************************************************************/

/******************************************************************************************
 Globals
******************************************************************************************/
__SYS_STACK_SIZE = 1K;

FLASH_START_ADDRESS     = 0x08000000;
FLASH_LENGTH            = 1M;

SRAM1_START_ADDRESS     = 0x20010000;
SRAM1_LENGTH            = 240K;

SRAM2_START_ADDRESS     = 0x2004c000;
SRAM2_LENGTH            = 16K;

SRAM_DTCM_START_ADDRESS = 0x20000000;
SRAM_DTCM_LENGTH        = 64K;

SRAM_ITCM_START_ADDRESS = 0x00000000;
SRAM_ITCM_LENGTH        = 16K;


/******************************************************************************************
 Memory configuration
******************************************************************************************/

MEMORY
{
  ROM(rx)  : ORIGIN = FLASH_START_ADDRESS, LENGTH = FLASH_LENGTH
  RAM(rwx) : ORIGIN = SRAM1_START_ADDRESS, LENGTH = SRAM1_LENGTH
}

/******************************************************************************************
 Sections definition
******************************************************************************************/
SECTIONS
{
  /* Interrupt vector table */
  .intvect : ALIGN(4)
  {
    PROVIDE(__INTVECT_BASE_ADDRESS = .) ;
    *(.intvect)
    KEEP(*(.intvect))
  } > ROM
  
  /* Program code (text) */
  .text : ALIGN(4)
  {
    *(.text)
  } > ROM

  /* Read-only data (.rodata) */
  .rodata : ALIGN(4)
  {
    *(.rodata)
  } > ROM

 /* Section for constructors */
  .ctors : ALIGN(4)
  {
    __CTOR_LIST__ = . ;
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    LONG(-1) ;
    __CTOR_END__ = . ;
  }  > ROM


  /* Section for destructors */
  .dtors : ALIGN(4)
  {
    __DTOR_LIST__ = . ;
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array))
    LONG(-1) ;
    __DTOR_END__ = . ;
  } > ROM

  /* Runtime clear table */
  .clear_sec : ALIGN(4)
  {
    PROVIDE(__RUNTIME_CLEAR_TABLE = .) ;
    LONG(0 + ADDR(.bss));   LONG(SIZEOF(.bss));
    LONG(0 + ADDR(.sbss));  LONG(SIZEOF(.sbss));
    LONG(0 + ADDR(.zbss));  LONG(SIZEOF(.zbss));
    LONG(0 + ADDR(.bbss));  LONG(SIZEOF(.bbss));
    LONG(-1);                 LONG(-1);
  } > ROM

  /* Runtime copy table */
  .copy_sec : ALIGN(4)
  {
    PROVIDE(__RUNTIME_COPY_TABLE = .) ;
    LONG(LOADADDR(.data));  LONG(0 + ADDR(.data));  LONG(SIZEOF(.data));
    LONG(LOADADDR(.sdata)); LONG(0 + ADDR(.sdata)); LONG(SIZEOF(.sdata));
    LONG(LOADADDR(.zdata)); LONG(0 + ADDR(.zdata)); LONG(SIZEOF(.zdata));
    LONG(LOADADDR(.bdata)); LONG(0 + ADDR(.bdata)); LONG(SIZEOF(.bdata));
    LONG(-1);                 LONG(-1);                  LONG(-1);
  } > ROM 

  /* The ROM-to-RAM initialized data section */
  .data : ALIGN(4)
  {
    *(.data)
  } > RAM  AT>ROM

  /* The uninitialized (zero-cleared) bss section */
  .bss : ALIGN(4)
  {
    *(.bss)
  } > RAM

  /* The ROM-to-RAM initialized sdata section */
  .sdata : ALIGN(4)
  {
    *(.sdata)
  } > RAM  AT>ROM

  /* The uninitialized (zero-cleared) sbss section */
  .sbss : ALIGN(4)
  {
    *(.sbss)
  } > RAM

  /* The ROM-to-RAM initialized zdata section */
  .zdata : ALIGN(4)
  {
    *(.zdata)
  } > RAM  AT>ROM

  /* The uninitialized (zero-cleared) zbss section */
  .zbss : ALIGN(4)
  {
    *(.zbss)
  } > RAM

  /* The ROM-to-RAM initialized bdata section */
  .bdata : ALIGN(4)
  {
    *(.bdata)
  } > RAM  AT>ROM

  /* The uninitialized (zero-cleared) bbss section */
  .bbss : ALIGN(4)
  {
    *(.sbss)
  } > RAM

  /* user-system mode stack definition */
  .stack :
  {
    . = ALIGN(MAX(__SYS_STACK_SIZE , .), 8);
    PROVIDE(__SYS_STACK_TOP = .) ;
  } > RAM
    
}
