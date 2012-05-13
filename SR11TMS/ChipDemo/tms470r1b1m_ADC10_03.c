//******************************************************************************
//  TMS470 Demo - MibADC Sample ADIN0 Continuous 32KSPS using interrupts
//
//  Description; A single sample is made on ADIN0 with reference to
//  AVcc/AVss. Software toggles HET after each conversion to reflect 8 bits
//  Software sets starts sample and conversion and an interrupt is generated
//  to signify end of conversion
//
//  Total sample and convert time = 19.6608 MHz/(8*((62+2)+11)) = 32.7KSPS
//  SYSCLK = MCLK = ACLK = 8 x 7.3728MHz = 58.9824MHz
//  ICLK = SYSCLK / 3 = 19.6608MHz
//
//  //*An external 7.3728MHz XTAL with proper load caps is required*//
//
//              TMS-FET470B1M
//             -----------------
//            |            OSCIN|-
//         +--|PLLDIS           | 7.3728MHz
//         |  |           OSCOUT|-
//        -+- |                 |
//            |                 |
//        >---|ADIN0         HET|---> Toggle
//            |                 |
//
//  L.Westlund / J.Mangino
//  Texas Instruments, Inc
//  July 29th 2005
//  Built with IAR Embedded Workbench Version: 4.30A
//******************************************************************************

#include <intrinsics.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include <TexasInstruments/tms470r1B1m_bit_definitions.h>

void LedSet(unsigned int mask);

int main(void)
{
  PCR = CLKDIV_3;                         // ICLK = SYSCLK / 3
  GCR = ZPLL_CLK_DIV_PRE_1;               // SYSCLK = 8 x fOSC
  PCR |= PENABLE;                         // Enable peripherals

  HETDIR  = 0xFFFFFFFF;                   // HETx Output direction
  HETDOUT = 0x00000000;

  GIODIRE = 0xFF;                         // GIO[E] set as outputs
  GIODOUTE = 0x00;

  ADCR1 |= PS_8;                          // ADCLK prescaler = 8
  ADSAMPEV |= SEN;                        // ADCSAMP1 controls SW
  ADSAMP1 = 62;                           // SW = 62+2

  ADCR1 |= ADC_EN;                        // Enable ADC
  ADCR2 |= G1_MODE;                       // Continuous Conversion
  ADISR1 = 0x0001;                        // Convert group 1 = channel 0

  REQMASK |= (1<<27);                     // enable channel 27 (AD1)
  ADCR2 |= ENA_GP1_INT;                   // enable group 1 interrupt

  __enable_interrupt();                   // enable interrupts

  // Loop forever.
  while(1){}
}

// display binary value on 8 LEDs
void LedSet(unsigned int mask)
{

  GIODOUTE = mask & 0xFF;
  HETDOUT = (mask & 0xFF00)>>8;

}


//------------------------------------------------------------------------------
// TMS470R1B1M Standard Interrupt Handler
//------------------------------------------------------------------------------

__irq __arm void IRQ_Handler(void)
{
   switch((0xff & IRQIVEC)-1)
  {
   case CIM_MIBADCE1  :                   // channel 27 (AD1) interrupt?
      LedSet(ADDR0);
      break;
  }
}

