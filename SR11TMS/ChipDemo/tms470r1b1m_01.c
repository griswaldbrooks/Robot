//*******************************************************************************
//  TMS470 Demo - Software Toggle GIOB0 Output
//
//  Description; Toggle pin GIOB0 inside of a software loop.
//
//  SYSCLK = MCLK = ACLK = 8 x 7.3728MHz = 58.9824MHz
//  ICLK = SYSCLK / 2 = 29.4912MHz
//
//  //*An external 7.3728MHz XTAL with proper load caps is required*//	
//
//              TMS-FET470B1M
//             -----------------
//            |            OSCIN|-
//            |                 | 7.3728MHz
//         +--|PLLDIS     OSCOUT|-
//         |  |                 |
//        -+- |            GIOB0|---> LED
//            |                 |
//
//  A.Dannenberg / J.Mangino
//  Texas Instruments, Inc
//  July 29th 2005
//  Built with IAR Embedded Workbench Version: 4.30A
//******************************************************************************

#include <intrinsics.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include <TexasInstruments/tms470r1B1m_bit_definitions.h>

int main(void)
{
  PCR = CLKDIV_2;                         // ICLK = SYSCLK / 2
  GCR = ZPLL_CLK_DIV_PRE_1;               // SYSCLK = 8 x fOSC
  PCR |= PENABLE;                         // Enable peripherals

  GIODCLRB = 0x00000001;                  // Reset GIOB0 output
  GIODIRB = 0x00000001;                   // GIOB0 to output direction

  for (;;)
  {
    volatile unsigned long i;
    GIODOUTB^= 0x00000001;                // GIOB0 Toggle
    for ( i = 0; i < 800000; i++ );
  }
}

//------------------------------------------------------------------------------
// TMS470R1B1M Standard Interrupt Handler
//------------------------------------------------------------------------------

__irq __arm void IRQ_Handler(void)
{
   switch((0xff & IRQIVEC)-1)
  {
  }
}

