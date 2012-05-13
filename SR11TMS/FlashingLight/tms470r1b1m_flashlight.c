
//*****************************************************************************
//  TMS470 Demo -  Flashing LED Sample Program
//
// This is a Program to flash the LEDs in a crossing pattern.
// Data from a table is displayed on the GIO and HET LEDs based on the RTI timing.
//
//                              TMS-FET470B1M
//                             _________________
//                            |                 |
//                         /|\|            OSCIN|-
//                          | |                 | 7.5MHz
//                          --|PLLDIS     OSCOUT|-
//                            |                 |
//                            |_________________|
//
//  J. Mangino/
//  Texas Instruments, Inc
//  August 2005
//  Built with IAR Embedded Workbench Version: 4.30A
//******************************************************************************

#include <intrinsics.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include <TexasInstruments/tms470r1B1m_bit_definitions.h>

// LED moving pattern codes.
static int led_table[] = {
  0x1818, 0x3c3c, 0x7e7e, 0xffff, 0xe7e7, 0xc3c3, 0x8181, 0x4242, 0x2424, -1
};
   int* ip;

void TMS470LedSet(unsigned int mask);
void COMP1_irq_handler();

int main(void)
{
  // Set up peripheral registers.
  // First disable interrupts.
  __disable_interrupt();
  ip = led_table;

  // Setup system.
  PCR = CLKDIV_4;                                      // ICLK = SYSCLK/4
  PCR |= PENABLE;                                      // enable peripherals
  GCR = ZPLL_CLK_DIV_PRE_1;               // SYSCLK = 8 x fOSC
  REQMASK = (1 << CIM_COMP1);        // Enable SPI Interrupt mask

  // Setup periodic interrupt using RTI with RTICMP1
  RTICNTEN = CNTEN_NOCNT;                       // Stop counting
  RTICNTR = 0x00;                           // clear 21-bits CNTR

  // Setup periodic interrupt timer
  // CMP1 used to generate  interrupt.
  RTIPCTL = 0x3;                         // preload 11-bits MOD
  RTICMP1 = 0xfffff;                     //
  RTICNTL = 0x00;                        // clear and disable tap

  // interrupt control, clear CMP1 and enable CMP1 interrupt
  RTICINT = 0x00;
  RTICINT |= CMP1ENA;

  // Start count, CNTR and MOD will count in both USER and SYSTEM mode
  RTICNTEN = CNTEN_UP;

  HETDIR  = 0xff;                        // Set HET as GIO outputs
  HETDOUT = 0xff;                        // Output on
  HETDOUT = 0x00;                        // Output off
  HETDOUT = 0xff;                        // Output on
  GIODIRE = 0xff;                        // Set GIO outputs
  GIODOUTE =0xff;                        // Output on

   __enable_interrupt();                 // Enable Interrupts

  // Loop forever.
  while (1);

}

  //------------------------------------------------------------------------------
// TMS470R1B1M Standard Interrupt Handler
//------------------------------------------------------------------------------

__irq __arm void IRQ_Handler(void)
{
   switch((0xff & IRQIVEC)-1)
  {
   case CIM_COMP1  : COMP1_irq_handler(); break;

  }
}


   void COMP1_irq_handler()
{
  RTICINT &= ~CMP1FLAG;   // interrupt control, clear CMP1

  if (*ip != -1)
  {
    TMS470LedSet(*ip);
    ip++;
}
 else
 {
    ip = led_table;
    TMS470LedSet(*ip);
    ip++;
  }
}


  void TMS470LedSet(unsigned int mask)
{
  GIODOUTE = mask & 0xFF;

  HETDOUT = (mask & 0xFF00)>>8;

}














