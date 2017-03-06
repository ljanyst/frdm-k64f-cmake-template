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

//------------------------------------------------------------------------------
// Register addresses
//------------------------------------------------------------------------------
#define STCTRL_REG    (*(volatile unsigned long *)0xe000e010)
#define STRELOAD_REG  (*(volatile unsigned long *)0xe000e014)
#define STCURRENT_REG (*(volatile unsigned long *)0xe000e018)

#define SIM_SCGC5_REG (*(volatile unsigned long *)0x40048038)

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTE 4

#define PORT_PCR_BASE   0x40049000
#define PORT_PCR_OFFSET 0x1000
#define PORT_PCR_REG(PORT, PIN) (*(volatile unsigned long*)(PORT_PCR_BASE + PORT*PORT_PCR_OFFSET + 4*PIN))

#define GPIO_BASE        0x400ff000
#define GPIO_PORT_OFFSET 0x40
#define PDDR 0x14
#define PDOR 0x00
#define PDIR 0x10
#define GPIO_REG(PORT, REG) (*(volatile unsigned long*)(GPIO_BASE + PORT*GPIO_PORT_OFFSET + REG))

//------------------------------------------------------------------------------
// Initialize the SysTick timer
//------------------------------------------------------------------------------
void init_sys_tick()
{
  STCTRL_REG    = 0;                  // disable
  STRELOAD_REG  = 0x00ffffff;         // maximum reload value
  STCURRENT_REG = 0;                  // clear the current
  STCTRL_REG    = 0x00000005;         // enable with the system clock
}

//------------------------------------------------------------------------------
// Initialize GPIO
//------------------------------------------------------------------------------
void init_gpio()
{
  // Enable the clock for ports A, B, C, and E. 
  SIM_SCGC5_REG |= 0x2e00;

  // Pin alternative selection
  PORT_PCR_REG(PORTB, 21) = 0x100; // Blue LED (GPIO)
  PORT_PCR_REG(PORTB, 22) = 0x100; // Red LED (GPIO)
  PORT_PCR_REG(PORTE, 26) = 0x100; // Green LED (GPIO)
  PORT_PCR_REG(PORTC, 6)  = 0x100; // SW2 (GPIO)
  PORT_PCR_REG(PORTA, 4)  = 0x100; // SW3 (GPIO)

  // Set the data direction
  GPIO_REG(PORTB, PDDR) |= (1 << 21); // PortB 21 - Output
  GPIO_REG(PORTB, PDDR) |= (1 << 22); // PortB 22 - Output
  GPIO_REG(PORTE, PDDR) |= (1 << 26); // PortE 26 - Output
  GPIO_REG(PORTC, PDDR) &= ~(1 << 6); // PortC  6 - Input
  GPIO_REG(PORTA, PDDR) &= ~(1 << 4); // PortA  4 - Input

  // Turn off the LEDs
  GPIO_REG(PORTB, PDOR) |= (1 << 21); // Blue
  GPIO_REG(PORTB, PDOR) |= (1 << 22); // Red
  GPIO_REG(PORTE, PDOR) |= (1 << 26); // Green
}

//------------------------------------------------------------------------------
// Count bus cycles and block until counted enough. By default the system clock
// runs at 22.5MHz this gives 44.4ns per tick.
//------------------------------------------------------------------------------
void st_wait_cycles(unsigned long delay)
{
  STRELOAD_REG = delay-1;
  STCURRENT_REG = 0;
  while(!(STCTRL_REG & 0x00010000));
}

//------------------------------------------------------------------------------
// Wait a multiple of one milisecond
//------------------------------------------------------------------------------
void st_wait(unsigned long times)
{
  int i;
  for(i = 0; i < times; ++i)
    st_wait_cycles(22500); // 22500 * 44.4ns = 1ms
}

//------------------------------------------------------------------------------
// Start the show
//------------------------------------------------------------------------------
int main(void)
{
  init_sys_tick();
  init_gpio();

  unsigned long ledB = 0;
  unsigned long ledE = 0;

  while(1) {
    unsigned long sw2 = !(GPIO_REG(PORTC, PDIR) & (1 << 6));
    unsigned long sw3 = !(GPIO_REG(PORTA, PDIR) & (1 << 4));
    if(sw2 && sw3) { ledB = (3 << 21); ledE = (1 << 26); }
    else if(sw2)   { ledB = (1 << 22); ledE = 0; }
    else if(sw3)   { ledB = (1 << 21); ledE = 0; }
    else           { ledB = 0; ledE = (1 << 26); }

    GPIO_REG(PORTB, PDOR) &= ~ledB;
    GPIO_REG(PORTE, PDOR) &= ~ledE;
    st_wait(1000);
    GPIO_REG(PORTB, PDOR) |= ledB;
    GPIO_REG(PORTE, PDOR) |= ledE;    
    st_wait(1000);
  }
}
