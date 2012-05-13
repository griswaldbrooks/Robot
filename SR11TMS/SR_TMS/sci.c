#include "sci.h"

void SCI1SendByte( char c ){
    while((SCI1CTL2 & 0x04) == 0x00); //If SC1 Tx buffer is not ready, wait
    SCI1TXBUF = c; 
}

void SCI1SendLFCR(){
	SCI1SendByte(0x0A);	//LF
	SCI1SendByte(0x0D); //CR
}