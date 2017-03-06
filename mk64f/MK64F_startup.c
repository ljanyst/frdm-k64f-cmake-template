//------------------------------------------------------------------------------
// Copyright (c) 2017 by Lukasz Janyst <lukasz@jany.st>
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED 'AS IS' AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
// REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
// INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
// LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
// OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
// PERFORMANCE OF THIS SOFTWARE.
//------------------------------------------------------------------------------

#include <stdint.h>

//------------------------------------------------------------------------------
// Handler declarations
//------------------------------------------------------------------------------
void __int_handler(void);
#define DEFINE_HANDLER(NAME)  void NAME ## _handler() __attribute__ ((weak, alias ("__int_handler")))

void __rst_handler();
void reset_handler() __attribute__ ((weak, alias ("__rst_handler")));

DEFINE_HANDLER(nmi);
DEFINE_HANDLER(hard_fault);
DEFINE_HANDLER(mman);
DEFINE_HANDLER(bus_fault);
DEFINE_HANDLER(usage_fault);
DEFINE_HANDLER(svcall);
DEFINE_HANDLER(debug_monitor);
DEFINE_HANDLER(pendsv);
DEFINE_HANDLER(systick);

DEFINE_HANDLER(dma0);
DEFINE_HANDLER(dma1);
DEFINE_HANDLER(dma2);
DEFINE_HANDLER(dma3);
DEFINE_HANDLER(dma4);
DEFINE_HANDLER(dma5);
DEFINE_HANDLER(dma6);
DEFINE_HANDLER(dma7);
DEFINE_HANDLER(dma8);
DEFINE_HANDLER(dma9);
DEFINE_HANDLER(dma10);
DEFINE_HANDLER(dma11);
DEFINE_HANDLER(dma12);
DEFINE_HANDLER(dma13);
DEFINE_HANDLER(dma14);
DEFINE_HANDLER(dma15);
DEFINE_HANDLER(dma_error);
DEFINE_HANDLER(flash_comm_complete);
DEFINE_HANDLER(flash_read_collision);
DEFINE_HANDLER(low_voltage);
DEFINE_HANDLER(llwu);
DEFINE_HANDLER(wdog);
DEFINE_HANDLER(rng);
DEFINE_HANDLER(i2c0);
DEFINE_HANDLER(i2c1);
DEFINE_HANDLER(spi0);
DEFINE_HANDLER(spi1);
DEFINE_HANDLER(i2s0_tx);
DEFINE_HANDLER(i2s0_rx);
DEFINE_HANDLER(uart0_status);
DEFINE_HANDLER(uart0_error);
DEFINE_HANDLER(uart1_status);
DEFINE_HANDLER(uart1_error);
DEFINE_HANDLER(uart2_status);
DEFINE_HANDLER(uart2_error);
DEFINE_HANDLER(uart3_status);
DEFINE_HANDLER(uart3_error);
DEFINE_HANDLER(adc0);
DEFINE_HANDLER(cmp0);
DEFINE_HANDLER(cmp1);
DEFINE_HANDLER(ftm0);
DEFINE_HANDLER(ftm1);
DEFINE_HANDLER(ftm2);
DEFINE_HANDLER(cmt);
DEFINE_HANDLER(rtc_alarm);
DEFINE_HANDLER(rtc_seconds);
DEFINE_HANDLER(pit0);
DEFINE_HANDLER(pit1);
DEFINE_HANDLER(pit2);
DEFINE_HANDLER(pit3);
DEFINE_HANDLER(pdb);
DEFINE_HANDLER(usb_otg);
DEFINE_HANDLER(usb_charger_detect);
DEFINE_HANDLER(dac0);
DEFINE_HANDLER(mcg);
DEFINE_HANDLER(low_power_timer);
DEFINE_HANDLER(pin_detect_porta);
DEFINE_HANDLER(pin_detect_portb);
DEFINE_HANDLER(pin_detect_portc);
DEFINE_HANDLER(pin_detect_portd);
DEFINE_HANDLER(pin_detect_porte);
DEFINE_HANDLER(software);
DEFINE_HANDLER(spi2);
DEFINE_HANDLER(uart4_status);
DEFINE_HANDLER(uart4_error);
DEFINE_HANDLER(uart5_status);
DEFINE_HANDLER(uart5_error);
DEFINE_HANDLER(cmp2);
DEFINE_HANDLER(ftm3);
DEFINE_HANDLER(dac1);
DEFINE_HANDLER(adc1);
DEFINE_HANDLER(i2c2);
DEFINE_HANDLER(can0_ored_msg);
DEFINE_HANDLER(can0_bus_off);
DEFINE_HANDLER(can0_error);
DEFINE_HANDLER(can0_tx_warning);
DEFINE_HANDLER(can0_rx_warning);
DEFINE_HANDLER(can0_wakeup);
DEFINE_HANDLER(sdhc);
DEFINE_HANDLER(ether_timer);
DEFINE_HANDLER(ether_tx);
DEFINE_HANDLER(ether_rx);
DEFINE_HANDLER(ether_error);

//------------------------------------------------------------------------------
// NVIC table
//------------------------------------------------------------------------------
#define HANDLER(NAME) NAME ## _handler
void(* nvic_table[])(void) __attribute__ ((section (".nvic"))) = {
  HANDLER(reset),
  HANDLER(nmi),
  HANDLER(hard_fault),
  HANDLER(mman),
  HANDLER(bus_fault),
  HANDLER(usage_fault),
  0, 0, 0, 0,
  HANDLER(svcall),
  HANDLER(debug_monitor),
  0,
  HANDLER(pendsv),
  HANDLER(systick),
  HANDLER(dma0),
  HANDLER(dma1),
  HANDLER(dma2),
  HANDLER(dma3),
  HANDLER(dma4),
  HANDLER(dma5),
  HANDLER(dma6),
  HANDLER(dma7),
  HANDLER(dma8),
  HANDLER(dma9),
  HANDLER(dma10),
  HANDLER(dma11),
  HANDLER(dma12),
  HANDLER(dma13),
  HANDLER(dma14),
  HANDLER(dma15),
  HANDLER(dma_error),
  HANDLER(flash_comm_complete),
  HANDLER(flash_read_collision),
  HANDLER(low_voltage),
  HANDLER(llwu),
  HANDLER(wdog),
  HANDLER(rng),
  HANDLER(i2c0),
  HANDLER(i2c1),
  HANDLER(spi0),
  HANDLER(spi1),
  HANDLER(i2s0_tx),
  HANDLER(i2s0_rx),
  0,
  HANDLER(uart0_status),
  HANDLER(uart0_error),
  HANDLER(uart1_status),
  HANDLER(uart1_error),
  HANDLER(uart2_status),
  HANDLER(uart2_error),
  HANDLER(uart3_status),
  HANDLER(uart3_error),
  HANDLER(adc0),
  HANDLER(cmp0),
  HANDLER(cmp1),
  HANDLER(ftm0),
  HANDLER(ftm1),
  HANDLER(ftm2),
  HANDLER(cmt),
  HANDLER(rtc_alarm),
  HANDLER(rtc_seconds),
  HANDLER(pit0),
  HANDLER(pit1),
  HANDLER(pit2),
  HANDLER(pit3),
  HANDLER(pdb),
  HANDLER(usb_otg),
  HANDLER(usb_charger_detect),
  0,
  HANDLER(dac0),
  HANDLER(mcg),
  HANDLER(low_power_timer),
  HANDLER(pin_detect_porta),
  HANDLER(pin_detect_portb),
  HANDLER(pin_detect_portc),
  HANDLER(pin_detect_portd),
  HANDLER(pin_detect_porte),
  HANDLER(software),
  HANDLER(spi2),
  HANDLER(uart4_status),
  HANDLER(uart4_error),
  HANDLER(uart5_status),
  HANDLER(uart5_error),
  HANDLER(cmp2),
  HANDLER(ftm3),
  HANDLER(dac1),
  HANDLER(adc1),
  HANDLER(i2c2),
  HANDLER(can0_ored_msg),
  HANDLER(can0_bus_off),
  HANDLER(can0_error),
  HANDLER(can0_tx_warning),
  HANDLER(can0_rx_warning),
  HANDLER(can0_wakeup),
  HANDLER(sdhc),
  HANDLER(ether_timer),
  HANDLER(ether_tx),
  HANDLER(ether_rx),
  HANDLER(ether_error)};

//------------------------------------------------------------------------------
// Flash Configuration Field
//------------------------------------------------------------------------------
static const struct {
  uint8_t backdor_key[8];   // backdor key
  uint8_t fprot[4];         // program flash protection (FPROT{0-3})
  uint8_t fsec;             // flash security (FSEC)
  uint8_t fopt;             // flash nonvolatile option (FOPT)
  uint8_t feprot;           // EEPROM protection (FEPROT)
  uint8_t fdprot;           // data flash protection (FDPROT)
} fcf  __attribute__ ((section (".fcf"))) = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0xff, 0xff, 0xff, 0xff}, // disable flash program protection
  0x02,                     // disable flash security
  0x01,                     // disable low-power boot
  0x00,
  0x00
};

//------------------------------------------------------------------------------
// Watchdog registers
//------------------------------------------------------------------------------
#define WDOG_UNLOCK  (*(volatile uint16_t *)0x4005200e)
#define WDOG_STCTRLH (*(volatile uint16_t *)0x40052000)
#define CPAC_REG     (*(volatile uint32_t *)0xe000ed88)

//------------------------------------------------------------------------------
// Linker symbols
//------------------------------------------------------------------------------
extern unsigned long __text_end_vma;
extern unsigned long __data_start_vma;
extern unsigned long __data_end_vma;
extern unsigned long __bss_start_vma;
extern unsigned long __bss_end_vma;

//------------------------------------------------------------------------------
// Handler definitions
//------------------------------------------------------------------------------
void __int_handler(void)
{
  while(1);
}

extern void main();

void __rst_handler()
{
  //----------------------------------------------------------------------------
  // Disable the reset watchdog
  //----------------------------------------------------------------------------
  WDOG_UNLOCK = 0xc520;        // unlock magic #1
  WDOG_UNLOCK = 0xd928;        // unlock magic #2
  for(int i = 0; i < 2; ++i);  // delay a couple of cycles
  WDOG_STCTRLH &= ~0x0001;     // disable the watchdog

  //----------------------------------------------------------------------------
  // Copy the required data to RAM
  //----------------------------------------------------------------------------
  unsigned long *src = &__text_end_vma;
  unsigned long *dst = &__data_start_vma;

  while(dst < &__data_end_vma) *dst++ = *src++;
  dst = &__bss_start_vma;
  while(dst < &__bss_end_vma) *dst++ = 0;
  
  //----------------------------------------------------------------------------
  // Set permissions to the floating point coprocessor
  //----------------------------------------------------------------------------
  CPAC_REG |= (0x0f << 20);
  __asm__ volatile (
    "dsb\r\n"        // force memory writes before continuing
    "isb\r\n" );     // reset the pipeline

  main();
}
