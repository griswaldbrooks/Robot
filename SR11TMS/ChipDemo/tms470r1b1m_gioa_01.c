//*******************************************************************************
//  TMS470 Demo - Software Poll GPIOA4/5/6/7 Toggling HET0/1/2/3
//
//  Description; Poll inputs GPIOA4/5/6/7, inverts the state, and sets the
//  HET0/1/2/3 outputs accordingly.
//
//  SYSCLK = MCLK = ACLK = 8 x 7.3728MHz = 58.9824MHz
//  ICLK = SYSCLK / 2 = 29.4912MHz
//
//  //*An external 7.3728MHz XTAL with proper load caps is required*//	
//
//              TMS-FET470B1M
//             -----------------
//         +--|PLLDIS      OSCIN|-
//         |  |                 | 7.3728MHz
//        -+- |           OSCOUT|-
//            |                 |
//        --->|GPIOA4       HET0|---> LED
//        --->|GPIOA5       HET1|---> LED
//        --->|GPIOA6       HET2|---> LED
//        --->|GPIOA7       HET3|---> LED
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

  HETDIR = 0xFFFFFFFF;                    // HETx Output direction

  for (;;)
  {
    HETDOUT = (GIODINA >> 4) ^ 0x0f;      // Shift bits, invert and output
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
