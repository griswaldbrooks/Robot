
#include <intrinsics.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include <TexasInstruments/tms470r1B1m_bit_definitions.h>
#include "lcd.h"

int main(void)
{
  int x_pos = 0;
  int x_pos_old = 0;
  PCR = CLKDIV_3;                         // ICLK = SYSCLK / 3
  GCR = ZPLL_CLK_DIV_PRE_1;               // SYSCLK = 8 x fOSC
  PCR |= PENABLE;                         // Enable peripherals

  HETDIR  = 0xFFFFFFFF;                   // HETx Output direction
  HETDOUT = 0x00000000;

  init_lcd();
  light_on();

  put_cursor( 0,0 );
  put_text( "0V" );
  put_cursor( 4,0 );
  put_text( "1V" );
  put_cursor( 9,0 );
  put_text( "2V" );
  put_cursor( 14,0 );
  put_text( "3V" );


  ADCR1 |= PS_8;                          // ADCLK prescaler = 8
  ADSAMPEV |= SEN;                        // ADCSAMP1 controls SW
  ADSAMP1 = 62;                           // SW = 62+2
  ADCR1 |= ADC_EN;                        // Enable ADC
  ADISR1 = 0x0006;                        // Convert croup 1 = channel 0
  //ADISR1 = 0x0001;                        // Convert croup 1 = channel 0
  ADCR2 |= G1_MODE;                       // Continuous Conversion

  for (;;)
  {
    while (!(ADSR & GP1_END));            // Wait for conversion to complete
    x_pos_old = x_pos;
    x_pos = ADDR2/64;
    //x_pos = ADDR0/64;
    if( x_pos_old != x_pos )
    {
      put_char(x_pos, 1, SQUARE);
      put_char(x_pos_old, 1, ' ');
    }
    ADSR |= GP1_END;                      // Clears flag
  }
}
