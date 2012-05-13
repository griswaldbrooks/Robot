//*******************************************************************************
//  TMS470 Demo - SCI2 19200 Echo With Interrupt
//
//  Description; Echo back received character in an endless loop. SCI2 RX
//  interrupt used.
//
//  SYSCLK = MCLK = ACLK = 8 x 7.3728MHz = 58.9824MHz
//  ICLK = SYSCLK / 2 = 29.4912MHz
//
//  Baud rate divider with 29.4912MHz ICLK @19200 =
//  29.4912MHz/(8*19200)-1 = 0xbf
//  Baud rate divider with 29.4912MHz ICLK @115200 =
//  29.4912MHz/(8*115200)-1 = 0x1f
//
//  //*An external 7.3728MHz XTAL with proper load caps is required*//	
//
//              TMS-FET470B1M
//             -----------------
//            |            OSCIN|-
//            |                 | 7.3728MHz
//         +--|PLLDIS     OSCOUT|-
//         |  |                 |
//        -+- |           SCI2TX|------------>
//            |                 | 19200 - 8N1
//            |           SCI2RX|<------------
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

  //Setup SCI2
  SCI2CTL3 &= ~SW_NRESET;                 // Reset SCI state machine
  SCI2CCR = TIMING_MODE_ASYNC + CHAR_8;   // Async, 8-bit Char
  SCI2CTL1 |= RXENA;                      // RX enabled
  SCI2CTL2 |= TXENA;                      // TX enabled
  SCI2CTL3 |= CLOCK + RX_ACTION_ENA;      // Internal clock. RX interrrupt
  SCI2LBAUD = 0x1f;                       // 29.4912MHz/(8*115200)-1
  //  SCI2LBAUD = 0xbf;                       // 29.4912MHz/(8*19200)-1
  SCI2PC2 |= RX_FUNC;                     // SCIRX is the SCI receive pin
  SCI2PC3 |= TX_FUNC;                     // SCITX is the SCI transmit pin
  SCI2CTL3 |= SW_NRESET;                  // Configure SCI2 state machine
  REQMASK = (1 << CIM_SCI2RX);            // Enable SCI2RX channel
  
  //Setup SCI1
  SCI1CTL3 &= ~SW_NRESET;                 // Reset SCI state machine
  SCI1CCR = TIMING_MODE_ASYNC + CHAR_8;   // Async, 8-bit Char
  SCI1CTL1 |= RXENA;                      // RX enabled
  SCI1CTL2 |= TXENA;                      // TX enabled
  SCI1CTL3 |= CLOCK + RX_ACTION_ENA;      // Internal clock. RX interrrupt
  SCI1LBAUD = 0x1f;                       // 29.4912MHz/(8*115200)-1
  SCI1PC2 |= RX_FUNC;                     // SCIRX is the SCI receive pin
  SCI1PC3 |= TX_FUNC;                     // SCITX is the SCI transmit pin
  SCI1CTL3 |= SW_NRESET;                  // Configure SCI2 state machine
  REQMASK = (1 << CIM_SCI1RX);            // Enable SCI2RX channel
  __enable_interrupt();                   // Enable interrupts


  SCI1TXBUF = 0x54; //Test message "TMS"
  SCI1TXBUF = 0x4D;
  while((SCI1CTL2 & 0x04) == 0x00);
  SCI1TXBUF = 0x53;
  
  for (;;) {}                             // Wait in enless loop
}
//------------------------------------------------------------------------------
// TMS470R1B1M Standard Interrupt Handler
//------------------------------------------------------------------------------

__irq __arm void IRQ_Handler(void)
{
  switch ((0xff & IRQIVEC) - 1)
  {
//    case CIM_SCI2RX : SCI2TXBUF = SCI2RXBUF; break;
      //Send SCI2 receive out on SCI1
      case CIM_SCI2RX : SCI1TXBUF = SCI2RXBUF; break;
  }
}
