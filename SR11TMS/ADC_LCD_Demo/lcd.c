#include <intrinsics.h>
#include <string.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include <TexasInstruments/tms470r1b1m_bit_definitions.h>
#include "lcd.h"
//******************************************************************************
//   LCD Library
//
//   Description;  Allows for easy use of most common LCD functions
//
//   L. Westlund / J. Mangino
//   Version    .1
//   Texas Instruments, Inc
//   September 23, 2005
//******************************************************************************
//Change log
//
// 0.1 - Inital Internal version - L.Westlund
//******************************************************************************
//------------------------------------------------------------------------------
// void put_text(char* t)
//
// This function places text onto the LCD at the position where the cursor is located
//
// IN:  t          The text to write
// OUT: -
//------------------------------------------------------------------------------
void put_text( char* t)
{
  int i;
  for(i=0; i < strlen( t ); i++)
  {
    write_data( t[i] );
  }
}
//------------------------------------------------------------------------------
// void put_char(int x, int y, int c)
//
// This function puts the cursor at given location on the LCD
// 0,0 is the top left position
// it then write the character given in the c variable at this location
//
// IN:  x          The X location, ranging from 0-15
//      y          The Y location, ranging from 0-1
//      c          The character to write to the display
// OUT: -
//------------------------------------------------------------------------------
void put_char(int x, int y, int c)
{
   put_cursor( x, y);
   write_data( c );
}
//------------------------------------------------------------------------------
// void put_cursor(int x, int y)
//
// This function puts the cursor at given location on the LCD
// 0,0 is the top left position
//
// IN:  x          The X location, ranging from 0-15
//      y          The Y location, ranging from 0-1
// OUT: -
//------------------------------------------------------------------------------
void put_cursor(int x, int y)
{
  int command = 0x80;                     // 'set cursor' instruction
  command |= x;
  if(y){ command |= 0x40; }
  write_control( command );
}
//------------------------------------------------------------------------------
// void read_data() - UNTESTED
//
// This function reads data from the LCD
//
// IN:  -
// OUT: LCD data
//------------------------------------------------------------------------------
int read_data()
{
  int ret = 0x00;                         // temp return variable

  GIODIRG &= ~0xF0;                       // Set Data lines as input to read data
  set_ena();
  ret = GIODING & 0xF0;                   // read first four bits (high bits)
  clr_ena();                              // clock in data;
  set_ena();
  ret = (GIODING & 0xF0)>>4;              // read second four bits (low bits)
  clr_ena();
  return ret;
}
//------------------------------------------------------------------------------
// void wait_lcd() - UNTESTED
//
// This function waits until the LCD has finished an operation
//
// IN:  -
// OUT: -
//------------------------------------------------------------------------------
void wait_lcd()
{
  while( read_data() == 0x80 ){}
}
//------------------------------------------------------------------------------
// void init_lcd()
//
// This function initializes the LCD to 4 bit mode and clears the screen
//
// IN:  -
// OUT: -
//------------------------------------------------------------------------------
void init_lcd()
{
  GIODIRG =0xff;
  GIODOUTG = 0x00;                        // set 8 bit mode
  set_ena();
  SW_Delay(DELAY_10MS);
  clr_ena();
  SW_Delay(DELAY_10MS);

  GIODOUTG = 0x20;                        // set 4 bit mode
  set_ena();
  SW_Delay(DELAY_10MS);
  clr_ena();
  SW_Delay(DELAY_10MS);

  write_control(0x28);
  write_control(0x0E);
  write_control(0x06);
  clear_lcd();
}
//------------------------------------------------------------------------------
// void write_control(int data)
//
// This function clocks in the variable data to the LCD as a command to the LCD
//
// IN:  data       variable containing 8 bits to write
// OUT: -
//------------------------------------------------------------------------------
void write_control(int data)
{
  clr_rs();
  write( data );
}
//------------------------------------------------------------------------------
// void write_data(int data)
//
// This function clocks in the variable data to the LCD in data format
//
// IN:  data       variable containing 8 bits to write
// OUT: -
//------------------------------------------------------------------------------
void write_data(int data)
{
  set_rs();
  write( data );
}
//------------------------------------------------------------------------------
// void write(int data)
//
// This function clocks in the variable data to the LCD
//
// IN:  data       variable containing 8 bits to write
// OUT: -
//------------------------------------------------------------------------------
void write( int data )
{
  GIODIRG |= 0xF0;                        // Set Data lines as output to write data
  set_io_data( data>>4 );                 // high nibble
  set_ena();
  SW_Delay(DELAY_200US);                  // Delay
  clr_ena();
  SW_Delay(DELAY_200US);                  // Delay
  set_io_data( data );                    // low nibble
  set_ena();
  SW_Delay(DELAY_200US);                  // Delay
  clr_ena();
  SW_Delay(DELAY_200US);                  // Delay
}
//------------------------------------------------------------------------------
// void set_io_data(int d)
//
// This function sets the io data lines for the LCD.
// The IO data lines become the value in the lower 4 bits of d
//
// IN:  d          variable containing lower 4 bits to write
// OUT: -
//------------------------------------------------------------------------------
void set_io_data( int d )
{
  GIODOUTG &= 0x0F;
  GIODOUTG |= (d&0x0F)<<4;
}
//These functions turn the LCD light line off/on
void light_on(void){GIODOUTG |= 0x04;}
void light_off(void){GIODOUTG &= ~0x04;}
//This function clears the LCD screen
void clear_lcd(){write_control(0x01);}
//These functions enable or disable the LCD line 'E'
void clr_ena(){GIODOUTG &= ~0x02;}
void set_ena(){GIODOUTG |=  0x02;}
//These functions enable or disable the LCD line 'R/W'
void clr_rw(){GIODOUTG &= ~0x04;}
void set_rw(){GIODOUTG |=  0x04;}
//These functions enable or disable the LCD line 'RS'
void clr_rs(){GIODOUTG &= ~0x08;}
void set_rs(){GIODOUTG |=  0x08;}
//------------------------------------------------------------------------------
// void SW_Delay(unsigned long Count)
//
// Function implements a compiler-independent software delay. The program
// execution is delayed by 4 x Count CPU clock cycles.
//
// IN:  Count      Acual Delay = 4 x Count
// OUT: -
//------------------------------------------------------------------------------
void SW_Delay(unsigned long Count)
{
  asm("       MOV          R1, R0         ");
  asm("       SUBS         R0, R1, #0x1   ");
  asm("       CMP          R1, #0x0       ");
  asm("       BNE          SW_Delay       ");
}
