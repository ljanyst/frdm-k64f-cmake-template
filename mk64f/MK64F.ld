/*------------------------------------------------------------------------------
 | Copyright (c) 2017 by Lukasz Janyst <lukasz@jany.st>
 |
 | Permission to use, copy, modify, and/or distribute this software for any
 | purpose with or without fee is hereby granted, provided that the above
 | copyright notice and this permission notice appear in all copies.
 |
 | THE SOFTWARE IS PROVIDED 'AS IS' AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 | REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 | AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 | INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 | LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 | OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 | PERFORMANCE OF THIS SOFTWARE.
 +----------------------------------------------------------------------------*/

MEMORY
{
  FLASH-NVIC (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  FLASH-FCF  (rx)  : ORIGIN = 0x00000400, LENGTH = 0x00000010
  FLASH      (rx)  : ORIGIN = 0x00000410, LENGTH = 0x0007fbef
  RAM1       (rwx) : ORIGIN = 0x1fff0000, LENGTH = 0x00010000
  RAM2       (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00030000
}

SECTIONS
{
  .nvic :
  {
    LONG(0x20030000) /* the initial stack location */
    KEEP(*(.nvic))
  } > FLASH-NVIC

  .fcf :
  {
    KEEP(*(.fcf))
  } > FLASH-FCF

  .text :
  {
    *(.text*)
    *(.rodata*)
  } > FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx*)
    *(.gnu.linkonce.armexidx*)
    __text_end_vma = .;
  } > FLASH

  .data :
  {
    __data_start_vma = .;
    *(.data*)
    __data_end_vma = .;
  } > RAM1 AT > FLASH

  .bss :
  {
    __bss_start_vma = .;
    *(.bss*)
    *(COMMON)	/* non-ISO C bulshit, just in case */
    __bss_end_vma = .;
  } > RAM1

  /DISCARD/ :
  {
    *(.init*)
    *(.fini*)
  }
}
