//******************************************************************************
//  TMS470R1A256 Demo - 2 x 115,200 Baud UART using HET, Interrupt driven
//
//  Description: This program implements a 2x115,200 Baud UARTs using the HET.
//  The received character are echo'ed back. Interrupt driven program flow is
//  used. Pins HET0/HET1 are used for UART1 and pins HET2/HET3 are used for
//  UART2. Note that any HET pins can be used.
//  SYSCLK = 8x7.5MHz = 60MHz.
//
//             TMS470R1B1M
//          +---------------+
//          |               |
//      +---|PLLDIS         |
//      |   |               |
//     ---  |               |
//     GND  |               |
//          |       XIN/XOUT|<--- 7.5MHz crystal
//          |               |
//          |           HET0|---> 115,200 Baud UART1
//          |           HET1|<---
//          |               |
//          |           HET2|---> 115,200 Baud UART2
//          |           HET3|<---
//          |               |
//          +---------------+
//
//  Andreas Dannenberg
//  Texas Instruments, Inc
//  June 06th 2006
//  Built with IAR Embedded Workbench Version: 4.31A
//  B1M EVM
//******************************************************************************

// Include Standard C Language Header Files
#include <stdlib.h>
#include <stdio.h>

// Include TMS470 Specific Header Files
#include <intrinsics.h>
#include "TexasInstruments\iotms470r1b1m.h"
#include "TexasInstruments\tms470r1b1m_bit_definitions.h"
#include "std_het.h"
#include "B1M_HET_UART_2X115200_H.h"

//Include Custom Header Files
#include "sci.h"
#include "het.h"

__no_init volatile HETPROGRAM0_UN e_HETPROGRAM0_UN @ 0x00800000;

//Globals
#define SET 0x01
#define UNSET 0x00

//unsigned int pwm1_duty = 0x0465;  //1125 Full Forward
unsigned int pwm1_duty = 0x0697;  //1687 Full Reverse
//unsigned int pwm1_duty = 0x057E;  //1406  Full Stop
//unsigned int pwm1_duty = 0x03A9;
unsigned int print_duty = UNSET;

//------------------------------------------------------------------------------
// Help function to load the HET program into the HET RAM using 32-bit access.
//------------------------------------------------------------------------------
void MemCopy32(unsigned long *dst, unsigned long *src, int bytes)
{
  for (int i = 0; i < (bytes + 3) / 4; i++)
    *dst++ = *src++;
}
//------------------------------------------------------------------------------
// void HetUART1PutByte(unsigned char Data)
//
// This function initializes the transmit one byte using the HET UART1.
// The transmit is performed ARM7 independent in the background by the
// HET program.
//------------------------------------------------------------------------------
void HetUART1PutByte(unsigned char Data)
{
  unsigned int Tmp = Data;

  Tmp <<= 1;                                    // Shift in start bit (0)
  Tmp |= 0x00000200;                            // Add stop bit (1)

  HET_TX1_Shift_0.memory.data_word = Tmp << 5;  // Load TX buffer
  HET_TX1_BitCtr_0.memory.data_word = 10 << 5;  // Load bit count
  HET_TX1_Start_0.memory.data_word = 0x1 << 5;  // Start TX
}
//------------------------------------------------------------------------------
// unsigned char HetUART1GetByte(void)
//
// This function reads out the last received character from the HET UART1.
//------------------------------------------------------------------------------
unsigned char HetUART1GetByte(void)
{
  unsigned int Tmp = HET_RX1_Buffer_0.memory.data_word >> 16;

  HET_RX1_Buffer_0.memory.data_word = 0;        // Clear RX buffer
  return Tmp;                                   // Return received byte
}
//------------------------------------------------------------------------------
// char HetUART1RxDataAvailable(void)
//
// This function checks if HET UART RX data is available and ready for
// read out.
//------------------------------------------------------------------------------
char HetUART1RxDataAvailable(void)
{
  unsigned int Tmp = HET_RX1_Buffer_0.memory.data_word >> 16;

  if (Tmp & 0x100)                              // Stop bit detected?
    return 1;                                   // Yes, data available
  else
    return 0;                                   // No data received
}
//------------------------------------------------------------------------------
// char HetUART1TxBufferEmpty(void)
//
// This function returns true if the HET UART1 TX buffer can be written to.
//------------------------------------------------------------------------------
char HetUART1TxBufferEmpty(void)
{
  if ((HET_TX1_BitCtr_0.memory.data_word >> 5) & 0xFFFFF)
    return 0;                                   // TX in progress
  else
    return 1;                                   // TX buffer empty
}
//------------------------------------------------------------------------------
// void HetUART2PutByte(unsigned char Data)
//
// This function initializes the transmit one byte using the HET UART2.
// The transmit is performed ARM7 independent in the background by the
// HET program.
//------------------------------------------------------------------------------
void HetUART2PutByte(unsigned char Data)
{
  unsigned int Tmp = Data;

  Tmp <<= 1;                                    // Shift in start bit (0)
  Tmp |= 0x00000200;                            // Add stop bit (1)

  HET_TX2_Shift_0.memory.data_word = Tmp << 5;  // Load TX buffer
  HET_TX2_BitCtr_0.memory.data_word = 10 << 5;  // Load bit count
  HET_TX2_Start_0.memory.data_word = 0x1 << 5;  // Start TX
}
//------------------------------------------------------------------------------
// unsigned char HetUART2GetByte(void)
//
// This function reads out the last received character from the HET UART2.
//------------------------------------------------------------------------------
unsigned char HetUART2GetByte(void)
{
  unsigned int Tmp = HET_RX2_Buffer_0.memory.data_word >> 16;

  HET_RX2_Buffer_0.memory.data_word = 0;        // Clear RX buffer
  return Tmp;                                   // Return received byte
}
//------------------------------------------------------------------------------
// char HetUART2RxDataAvailable(void)
//
// This function checks if HET UART RX data is available and ready for
// read out.
//------------------------------------------------------------------------------
char HetUART2RxDataAvailable(void)
{
  unsigned int Tmp = HET_RX2_Buffer_0.memory.data_word >> 16;

  if (Tmp & 0x100)                              // Stop bit detected?
    return 1;                                   // Yes, data available
  else
    return 0;                                   // No data received
}
//------------------------------------------------------------------------------
// char HetUART2TxBufferEmpty(void)
//
// This function returns true if the HET UART2 TX buffer can be written to.
//------------------------------------------------------------------------------
char HetUART2TxBufferEmpty(void)
{
  if ((HET_TX2_BitCtr_0.memory.data_word >> 5) & 0xFFFFF)
    return 0;                                   // TX in progress
  else
    return 1;                                   // TX buffer empty
}
//------------------------------------------------------------------------------

void HETuart_setup(){

  HETDOUT = 0x00000005;                         // Output HIGH on HET0/HET2
                                                // (RS-232 idle state)
  HETDIR  = 0xFFFFFFF5;                         // Set all HETx
                                                // but HET1/HET3 as outputs

//  REQMASK = (1 << CIM_HET1);                    // Enable HET level 1 ints
    REQMASK |= (1 << CIM_HET1);                    // Enable HET level 1 ints

  // Copy HET instructions to HET RAM
  MemCopy32((void *) &e_HETPROGRAM0_UN, (void *) HET_INIT0_PST,
            sizeof(HET_INIT0_PST));

  HETGCR = CLK_MASTER + IGNORE_SUSPEND;         // HET Master Mode,
                                                // Ignore SW BP
  HETPRY = (0x1 << 18) + (0x1 << 2) +           // Interrupts on instructions
           (0x1 << 27) + (0x1 << 11) +          // 18, 2, 27, 11(43)
           (0x1 << 19);                         // 19 (51)
                                                // are high priority

  
  // Set HET PFR register
  // fSYSCLK = 60MHz
  // fHR = fSYSCLK / 2 = 30Mhz
  // fLR = fHR / 32 = 937.5kHz
  HETPFR = LRPRES_FACTOR_32 + HRPRES_FACTOR_2;

 // __enable_interrupt() must be called at some point after this

  HETGCR |= ON;                                 // Enable HET
}

void SCI_setup(){

  //Setup SCI2
  SCI2CTL3 &= ~SW_NRESET;                 // Reset SCI state machine
  SCI2CCR = TIMING_MODE_ASYNC + CHAR_8;   // Async, 8-bit Char
  SCI2CTL1 |= RXENA;                      // RX enabled
  SCI2CTL2 |= TXENA;                      // TX enabled
  SCI2CTL3 |= CLOCK + RX_ACTION_ENA;      // Internal clock. RX interrrupt
  SCI2LBAUD = 0x1f;                       // 29.4912MHz/(8*115200)-1
  SCI2PC2 |= RX_FUNC;                     // SCIRX is the SCI receive pin
  SCI2PC3 |= TX_FUNC;                     // SCITX is the SCI transmit pin
  SCI2CTL3 |= SW_NRESET;                  // Configure SCI2 state machine
  REQMASK |= (1 << CIM_SCI2RX);            // Enable SCI2RX channel
  
  //Setup SCI1
  SCI1CTL3 &= ~SW_NRESET;                 // Reset SCI state machine
  SCI1CCR = TIMING_MODE_ASYNC + CHAR_8;   // Async, 8-bit Char
  SCI1CTL1 |= RXENA;                      // RX enabled
  SCI1CTL2 |= TXENA;                      // TX enabled
  SCI1CTL3 |= CLOCK + RX_ACTION_ENA;      // Internal clock. RX interrrupt
//  SCI1LBAUD = 0x07;                       // 29.4912MHz/(8*460800)-1
  SCI1LBAUD = 0x1f;                       // 29.4912MHz/(8*115200)-1
  SCI1PC2 |= RX_FUNC;                     // SCIRX is the SCI receive pin
  SCI1PC3 |= TX_FUNC;                     // SCITX is the SCI transmit pin
  SCI1CTL3 |= SW_NRESET;                  // Configure SCI2 state machine

  REQMASK |= (1 << CIM_SCI1RX);            // Enable SCI2RX channel
 // __enable_interrupt() must be called at some point after this

}

void LED_setup(){
  GIODIRE = 0xff;                        // Set GIO outputs
  GIODOUTE =0xff;                        // Output on
  GIODOUTE =0x00;                        // Output off
}

void ADC_setup(){
  
  ADCR1 |= PS_8;                          // ADCLK prescaler = 8
  ADSAMPEV |= SEN;                        // ADCSAMP1 controls SW
  ADSAMP1 = 62;                           // SW = 62+2
  ADCR1 |= ADC_EN;                        // Enable ADC
  ADISR1 = 0x0006;                        // Convert group 1 = channel 1 and 2
  //ADISR1 = 0x0001;                        // Convert group 1 = channel 0
  ADCR2 |= G1_MODE;                       // Continuous Conversion
}

int ADC_returnADIN1(){
  //assumes use of ADC_setup()
  return ADDR1;
}
int ADC_returnADIN2(){
  //assumes use of ADC_setup()
  return ADDR2;
}

int main(void)
{
  PCR = 0x00;                                   // Disable peripherals
  GCR = 0x00;                                   // Enable PLL multiply-by-8
  PCR = CLKDIV_2;                               // ICLK=SYSCLK/2
                                                // (to not violate max. ICLK)
  PCR |= PENABLE;                               // Enable peripherals
                                                // AFTER setting CLKDIV

  //PCR = CLKDIV_2;                         // ICLK = SYSCLK / 2
  //GCR = ZPLL_CLK_DIV_PRE_1;               // SYSCLK = 8 x fOSC
  //PCR |= PENABLE;                         // Enable peripherals
  
  HETuart_setup();
  SCI_setup();
  LED_setup();
  ADC_setup();
  __enable_interrupt();                   // Enable interrupts
  while (1)                                     // Loop forever...
  {
    
    GIODOUTE = ADC_returnADIN1();
    if(print_duty){
      printf("DUTY: %d\n", pwm1_duty);
      print_duty = UNSET;
    }
    
  }
}
//------------------------------------------------------------------------------
// HET UART1 Tx Handler
//
// This ISR is executed upon HET UART1 TX completion and can be used to transmit
// the next data byte.
//------------------------------------------------------------------------------
void HetUART1TxHandler()
{
}
//------------------------------------------------------------------------------
// HET UART1 Rx Handler
//
// This ISR is executed upon HET UART1 data receiption.
//------------------------------------------------------------------------------
void HetUART1RxHandler(void)
{
  //if (HetUART1TxBufferEmpty())
//    HetUART1PutByte(HetUART1GetByte());
    SCI1SendByte(HetUART1GetByte());
    //GIODOUTE = 0xff;
}
//------------------------------------------------------------------------------
// HET UART2 Tx Handler
//
// This ISR is executed upon HET UART2 TX completion and can be used to transmit
// the next data byte.
//------------------------------------------------------------------------------
void HetUART2TxHandler()
{
}
//------------------------------------------------------------------------------
// HET UART2 Rx Handler
//
// This ISR is executed upon HET UART2 data receiption.
//------------------------------------------------------------------------------
void HetUART2RxHandler(void)
{
  if (HetUART2TxBufferEmpty())
    HetUART2PutByte(HetUART2GetByte());
}
//------------------------------------------------------------------------------
// This ISR is executed upon HET PWM1 activation.
//------------------------------------------------------------------------------
void HetPWM1Handler(void)
{
   HET_PWM1_Compare_0.ecmp.data = (pwm1_duty >> 2);
}
//------------------------------------------------------------------------------
// HET Interrupt Level 1 Handler
//------------------------------------------------------------------------------
void HET1_irq_handler()
{

  switch ((HETOFF1 & 0xff) - 1)
  {
    case 2 :                                    // Int on HET instruction 2
      HetUART1TxHandler();
      break;
    case 11 :                                   // Int on HET instruction 11(43)
      HetUART2RxHandler();
      break;
    case 18 :                                   // Int on HET instruction 18
      HetUART1RxHandler();
      break;
    case 27 :                                   // Int on HET instruction 27
      HetUART2TxHandler();
      break;
    case 19 :                                   // Int on HET instruction 19 (51)
      HetPWM1Handler();
      break;
  }
}
//------------------------------------------------------------------------------
// TMS470R1B1M Standard Interrupt Handler
//------------------------------------------------------------------------------


__irq __arm void IRQ_Handler(void)
{
  switch ((0xff & IRQIVEC) - 1)
  {
      case CIM_HET1 :
        HET1_irq_handler();
        break;
    
//    case CIM_SCI2RX : SCI2TXBUF = SCI2RXBUF; break;
      //Send SCI2 receive out on SCI1
      case CIM_SCI2RX : 
        //SCI1TXBUF = SCI2RXBUF; 
        break;
      case CIM_SCI1RX : 
        //Echo and newline
        SCI1SendByte(SCI1RXBUF); 
        SCI1SendLFCR();
        if(SCI1RXBUF == 'p'){  
          pwm1_duty++;
        }
        if(SCI1RXBUF == 'l'){
          pwm1_duty--;
        }
        if(SCI1RXBUF == 'q'){
          print_duty = SET;
        }
        //GIODOUTE = 0x00;
        break;
  }
}