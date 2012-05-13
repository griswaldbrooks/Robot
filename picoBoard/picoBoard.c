//AVR includes
#include <avr/io.h>		    // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <stdio.h>			// stuff
#include <stdlib.h>			// stuff
#include <math.h>			// stuff
//#include "libm.a"			// required with math.h
#include <string.h>			// allow strings to be used
#include <avr/eeprom.h>		// adds EEPROM functionality



unsigned char i;
void f(){
	/* Define pull-ups and set outputs high */
	/* Define directions for port pins */
	PORTB = (1<<PORTB7)|(1<<PORTB6)|(1<<PORTB1)|(1<<PORTB0);
	DDRB = (1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);
	/* Insert nop for synchronization*/
	
	asm volatile("nop\n\t");
	//__no_operation();
	/* Read port pins */

	//i = PINB;
}

int main(){

f();

}
