/*
		This file has been auto-generate by WebbotLib tools V1.1
				** DO NOT MODIFY BY HAND **
*/
#ifndef _XHARDWARE_H_
#define _XHARDWARE_H_

#if !defined (_HARDWARE_H_)
#define BUILDING_LIBRARY
#define _HARDWARE_H
#endif

#ifndef F_CPU
#warning No CPU speed specified - assuming running at 16000000
#define F_CPU 16000000
#endif

#if F_CPU != 16000000
# warning "Board runs at 16000000 but you have defined a different value"
#endif

#if defined (__AVR_ATmega640__)
#else
#	error You must set the device to the ATmega640
#endif

#define FLASH_SIZE 65536
#define RAM_SIZE 8192
#define EEPROM_SIZE 4096
#define _NUM_PCINT_PINS 24

// Include library files
#include <Sensors/Distance/Sharp/GP2.h>
#include <Sensors/Encoder/Generic/quadrature.h>
#include <a2d.h>
#include <avr/eeprom.h>
#include <errors.h>
#include <libdefs.h>
#include <rprintf.h>
#include <servos.h>
#include <switch.h>
#include <timer.h>
#include <uart.h>

// ----------- Define the used pins ------------
extern IOPin PROGMEM _B6;
extern IOPin PROGMEM _C2;
extern IOPin PROGMEM _C3;
extern IOPin PROGMEM _C4;
extern IOPin PROGMEM _D2;
extern IOPin PROGMEM _D3;
extern IOPin PROGMEM _E0;
extern IOPin PROGMEM _E1;
extern IOPin PROGMEM _E2;
extern IOPin PROGMEM _E3;
extern IOPin PROGMEM _E4;
extern IOPin PROGMEM _E5;
extern IOPin PROGMEM _E7;
extern IOPin PROGMEM _G5;
extern IOPin PROGMEM _H3;
extern IOPin PROGMEM _H4;
extern IOPin PROGMEM _H5;
extern IOPin PROGMEM _H6;
extern IOPin PROGMEM _K4;
extern IOPin PROGMEM _K5;
extern IOPin PROGMEM _K6;
extern IOPin PROGMEM _K7;
#define B6 &_B6
#define C2 &_C2
#define C3 &_C3
#define C4 &_C4
#define D2 &_D2
#define D3 &_D3
#define E0 &_E0
#define E1 &_E1
#define E2 &_E2
#define E3 &_E3
#define E4 &_E4
#define E5 &_E5
#define E7 &_E7
#define G5 &_G5
#define H3 &_H3
#define H4 &_H4
#define H5 &_H5
#define H6 &_H6
#define K4 &_K4
#define K5 &_K5
#define K6 &_K6
#define K7 &_K7

#ifndef BUILDING_LIBRARY
#include <avr/io.h>
// ----------- Define the used ports ------------
static IOPort PROGMEM _PORTB={_SFR_MEM_ADDR(PORTB),_SFR_MEM_ADDR(DDRB),_SFR_MEM_ADDR(PINB)};
static IOPort PROGMEM _PORTC={_SFR_MEM_ADDR(PORTC),_SFR_MEM_ADDR(DDRC),_SFR_MEM_ADDR(PINC)};
static IOPort PROGMEM _PORTD={_SFR_MEM_ADDR(PORTD),_SFR_MEM_ADDR(DDRD),_SFR_MEM_ADDR(PIND)};
static IOPort PROGMEM _PORTE={_SFR_MEM_ADDR(PORTE),_SFR_MEM_ADDR(DDRE),_SFR_MEM_ADDR(PINE)};
static IOPort PROGMEM _PORTG={_SFR_MEM_ADDR(PORTG),_SFR_MEM_ADDR(DDRG),_SFR_MEM_ADDR(PING)};
static IOPort PROGMEM _PORTH={_SFR_MEM_ADDR(PORTH),_SFR_MEM_ADDR(DDRH),_SFR_MEM_ADDR(PINH)};
static IOPort PROGMEM _PORTK={_SFR_MEM_ADDR(PORTK),_SFR_MEM_ADDR(DDRK),_SFR_MEM_ADDR(PINK)};

// ----------- Define the used pins ------------
IOPin PROGMEM _B6 = { &_PORTB, BV(6)};
IOPin PROGMEM _C2 = { &_PORTC, BV(2)};
IOPin PROGMEM _C3 = { &_PORTC, BV(3)};
IOPin PROGMEM _C4 = { &_PORTC, BV(4)};
IOPin PROGMEM _D2 = { &_PORTD, BV(2)};
IOPin PROGMEM _D3 = { &_PORTD, BV(3)};
IOPin PROGMEM _E0 = { &_PORTE, BV(0)};
IOPin PROGMEM _E1 = { &_PORTE, BV(1)};
IOPin PROGMEM _E2 = { &_PORTE, BV(2)};
IOPin PROGMEM _E3 = { &_PORTE, BV(3)};
IOPin PROGMEM _E4 = { &_PORTE, BV(4)};
IOPin PROGMEM _E5 = { &_PORTE, BV(5)};
IOPin PROGMEM _E7 = { &_PORTE, BV(7)};
IOPin PROGMEM _G5 = { &_PORTG, BV(5)};
IOPin PROGMEM _H3 = { &_PORTH, BV(3)};
IOPin PROGMEM _H4 = { &_PORTH, BV(4)};
IOPin PROGMEM _H5 = { &_PORTH, BV(5)};
IOPin PROGMEM _H6 = { &_PORTH, BV(6)};
IOPin PROGMEM _K4 = { &_PORTK, BV(4)};
IOPin PROGMEM _K5 = { &_PORTK, BV(5)};
IOPin PROGMEM _K6 = { &_PORTK, BV(6)};
IOPin PROGMEM _K7 = { &_PORTK, BV(7)};
const uint8_t NUM_PCINT_PINS = 24;
const IOPin* PROGMEM PCINT_PINS[]={null,null,null,null,null,null,B6,null,E0,null,null,null,null,null,null,null,null,null,null,null,K4,K5,K6,K7};
#endif

// ------------------- uart1 -------------------

// Create hardware UART uart1
extern HW_UART _uart1;
#define uart1 &_uart1

// Create a routine to write bytes to uart1
// You can set rprintf to use it by calling rprintfInit(&uart1SendByte)
extern MAKE_WRITER(uart1SendByte);

// Create a routine to read a byte from uart1
// Returns -1 if there was no data
extern MAKE_READER( uart1GetByte);

// ------------------- bluetooth -------------------

// Create hardware UART bluetooth
extern HW_UART _bluetooth;
#define bluetooth &_bluetooth

// Create a routine to write bytes to bluetooth
// You can set rprintf to use it by calling rprintfInit(&bluetoothSendByte)
extern MAKE_WRITER(bluetoothSendByte);

// Create a routine to read a byte from bluetooth
// Returns -1 if there was no data
extern MAKE_READER( bluetoothGetByte);
// ----------- Start Timer Definition -------------

// Define timer 0
extern TimerCompare PROGMEM __timer0Compare[];
#define TIMER0_COMPAREA &__timer0Compare[0]
#define TIMER0_COMPAREB &__timer0Compare[1]

// Define timer 1
extern TimerCompare PROGMEM __timer1Compare[];
#define TIMER1_COMPAREA &__timer1Compare[0]
#define TIMER1_COMPAREB &__timer1Compare[1]
#define TIMER1_COMPAREC &__timer1Compare[2]

// Define timer 2
extern TimerCompare PROGMEM __timer2Compare[];
#define TIMER2_COMPAREA &__timer2Compare[0]
#define TIMER2_COMPAREB &__timer2Compare[1]

// Define timer 3
extern TimerCompare PROGMEM __timer3Compare[];
#define TIMER3_COMPAREA &__timer3Compare[0]
#define TIMER3_COMPAREB &__timer3Compare[1]
#define TIMER3_COMPAREC &__timer3Compare[2]

// Define timer 4
extern TimerCompare PROGMEM __timer4Compare[];
#define TIMER4_COMPAREA &__timer4Compare[0]
#define TIMER4_COMPAREB &__timer4Compare[1]
#define TIMER4_COMPAREC &__timer4Compare[2]

// Define timer 5
extern TimerCompare PROGMEM __timer5Compare[];
#define TIMER5_COMPAREA &__timer5Compare[0]
#define TIMER5_COMPAREB &__timer5Compare[1]
#define TIMER5_COMPAREC &__timer5Compare[2]

// Create table of timers
extern const PROGMEM Timer PROGMEM pgm_Timers[];
extern const uint8_t NUMBER_OF_TIMERS;
#define TIMER0 &pgm_Timers[0]
#define TIMER1 &pgm_Timers[1]
#define TIMER2 &pgm_Timers[2]
#define TIMER3 &pgm_Timers[3]
#define TIMER4 &pgm_Timers[4]
#define TIMER5 &pgm_Timers[5]
// ----------- End Timer Definition -------------


#ifndef BUILDING_LIBRARY
// Type of interrupt handler to use for timer interrupts.
// Do not change unless you know what you're doing.
#ifndef TIMER_INTERRUPT_HANDLER
#define TIMER_INTERRUPT_HANDLER	ISR
#endif
// ----------- Start Timer Definition -------------

// Define timer 0
static TimerData __timer0_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer0CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer0CompareB_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer0Compare[] = {
	MAKE_TIMER_COMPARE(__timer0CompareA_data,0,TIMSK0,OCIE0A,OCR0A,TIFR0,OCF0A,TCCR0A,COM0A0,null),
	MAKE_TIMER_COMPARE(__timer0CompareB_data,0,TIMSK0,OCIE0B,OCR0B,TIFR0,OCF0B,TCCR0A,COM0B0,G5)
};

// Define timer 1
static TimerData __timer1_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer1CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer1CompareB_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer1CompareC_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer1Compare[] = {
	MAKE_TIMER_COMPARE(__timer1CompareA_data,1,TIMSK1,OCIE1A,OCR1A,TIFR1,OCF1A,TCCR1A,COM1A0,null),
	MAKE_TIMER_COMPARE(__timer1CompareB_data,1,TIMSK1,OCIE1B,OCR1B,TIFR1,OCF1B,TCCR1A,COM1B0,B6),
	MAKE_TIMER_COMPARE(__timer1CompareC_data,1,TIMSK1,OCIE1C,OCR1C,TIFR1,OCF1C,TCCR1A,COM1C0,null)
};

// Define timer 2
static TimerData __timer2_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer2CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer2CompareB_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer2Compare[] = {
	MAKE_TIMER_COMPARE(__timer2CompareA_data,2,TIMSK2,OCIE2A,OCR2A,TIFR2,OCF2A,TCCR2A,COM2A0,null),
	MAKE_TIMER_COMPARE(__timer2CompareB_data,2,TIMSK2,OCIE2B,OCR2B,TIFR2,OCF2B,TCCR2A,COM2B0,H6)
};

// Define timer 3
static TimerData __timer3_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer3CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer3CompareB_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer3CompareC_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer3Compare[] = {
	MAKE_TIMER_COMPARE(__timer3CompareA_data,3,TIMSK3,OCIE3A,OCR3A,TIFR3,OCF3A,TCCR3A,COM3A0,E3),
	MAKE_TIMER_COMPARE(__timer3CompareB_data,3,TIMSK3,OCIE3B,OCR3B,TIFR3,OCF3B,TCCR3A,COM3B0,E4),
	MAKE_TIMER_COMPARE(__timer3CompareC_data,3,TIMSK3,OCIE3C,OCR3C,TIFR3,OCF3C,TCCR3A,COM3C0,E5)
};

// Define timer 4
static TimerData __timer4_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer4CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer4CompareB_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer4CompareC_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer4Compare[] = {
	MAKE_TIMER_COMPARE(__timer4CompareA_data,4,TIMSK4,OCIE4A,OCR4A,TIFR4,OCF4A,TCCR4A,COM4A0,H3),
	MAKE_TIMER_COMPARE(__timer4CompareB_data,4,TIMSK4,OCIE4B,OCR4B,TIFR4,OCF4B,TCCR4A,COM4B0,H4),
	MAKE_TIMER_COMPARE(__timer4CompareC_data,4,TIMSK4,OCIE4C,OCR4C,TIFR4,OCF4C,TCCR4A,COM4C0,H5)
};

// Define timer 5
static TimerData __timer5_data = MAKE_TIMER_DATA(0);
static TimerDataCompare __timer5CompareA_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer5CompareB_data = MAKE_TIMER_COMPARE_DATA();
static TimerDataCompare __timer5CompareC_data = MAKE_TIMER_COMPARE_DATA();
TimerCompare PROGMEM __timer5Compare[] = {
	MAKE_TIMER_COMPARE(__timer5CompareA_data,5,TIMSK5,OCIE5A,OCR5A,TIFR5,OCF5A,TCCR5A,COM5A0,null),
	MAKE_TIMER_COMPARE(__timer5CompareB_data,5,TIMSK5,OCIE5B,OCR5B,TIFR5,OCF5B,TCCR5A,COM5B0,null),
	MAKE_TIMER_COMPARE(__timer5CompareC_data,5,TIMSK5,OCIE5C,OCR5C,TIFR5,OCF5C,TCCR5A,COM5C0,null)
};

// Create table of timers
const PROGMEM Timer PROGMEM pgm_Timers[] = {
	MAKE_TIMER(__timer0_data,TCNT0,TCCR0B,false,false,TIMER_3BIT_MODES,
		TCCR0A,WGM00,TCCR0A,WGM01,TCCR0B,WGM02,UNUSED_PORT,0,
		__timer0Compare,TIMSK0,TOIE0,TIFR0,TOV0,UNUSED_PORT,UNUSED_PORT,0,UNUSED_PORT,0,UNUSED_PORT,0,null),
	MAKE_TIMER(__timer1_data,TCNT1,TCCR1B,true,false,TIMER_ALL_MODES,
		TCCR1A,WGM10,TCCR1A,WGM11,TCCR1B,WGM12,TCCR1B,WGM13,
		__timer1Compare,TIMSK1,TOIE1,TIFR1,TOV1,ICR1,TIMSK1,ICIE1,TIFR1,ICF1,TCCR1B,ICES1,null),
	MAKE_TIMER(__timer2_data,TCNT2,TCCR2B,false,true,TIMER_3BIT_MODES,
		TCCR2A,WGM20,TCCR2A,WGM21,TCCR2B,WGM22,UNUSED_PORT,0,
		__timer2Compare,TIMSK2,TOIE2,TIFR2,TOV2,UNUSED_PORT,UNUSED_PORT,0,UNUSED_PORT,0,UNUSED_PORT,0,null),
	MAKE_TIMER(__timer3_data,TCNT3,TCCR3B,true,false,TIMER_ALL_MODES,
		TCCR3A,WGM30,TCCR3A,WGM31,TCCR3B,WGM32,TCCR3B,WGM33,
		__timer3Compare,TIMSK3,TOIE3,TIFR3,TOV3,ICR3,TIMSK3,ICIE3,TIFR3,ICF3,TCCR3B,ICES3,E7),
	MAKE_TIMER(__timer4_data,TCNT4,TCCR4B,true,false,TIMER_ALL_MODES,
		TCCR4A,WGM40,TCCR4A,WGM41,TCCR4B,WGM42,TCCR4B,WGM43,
		__timer4Compare,TIMSK4,TOIE4,TIFR4,TOV4,ICR4,TIMSK4,ICIE4,TIFR4,ICF4,TCCR4B,ICES4,null),
	MAKE_TIMER(__timer5_data,TCNT5,TCCR5B,true,false,TIMER_ALL_MODES,
		TCCR5A,WGM50,TCCR5A,WGM51,TCCR5B,WGM52,TCCR5B,WGM53,
		__timer5Compare,TIMSK5,TOIE5,TIFR5,TOV5,ICR5,TIMSK5,ICIE5,TIFR5,ICF5,TCCR5B,ICES5,null)
};
const uint8_t NUMBER_OF_TIMERS =  (sizeof(pgm_Timers)/sizeof(Timer));

#ifdef TIMER0_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER0_OVF_vect){
	__timer_overflowService(TIMER0);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER0_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER0_COMPA_vect){
	__timer_compareService(TIMER0_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER0_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER0_COMPB_vect){
	__timer_compareService(TIMER0_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER1_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER1_OVF_vect){
	__timer_overflowService(TIMER1);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER1_CAPT_vect
TIMER_INTERRUPT_HANDLER(TIMER1_CAPT_vect){
	__timer_captureService(TIMER1);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER1_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER1_COMPA_vect){
	__timer_compareService(TIMER1_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER1_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER1_COMPB_vect){
	__timer_compareService(TIMER1_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER1_COMPC_vect
TIMER_INTERRUPT_HANDLER(TIMER1_COMPC_vect){
	__timer_compareService(TIMER1_COMPAREC);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER2_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER2_OVF_vect){
	__timer_overflowService(TIMER2);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER2_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER2_COMPA_vect){
	__timer_compareService(TIMER2_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER2_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER2_COMPB_vect){
	__timer_compareService(TIMER2_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER3_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER3_OVF_vect){
	__timer_overflowService(TIMER3);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER3_CAPT_vect
TIMER_INTERRUPT_HANDLER(TIMER3_CAPT_vect){
	__timer_captureService(TIMER3);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER3_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER3_COMPA_vect){
	__timer_compareService(TIMER3_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER3_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER3_COMPB_vect){
	__timer_compareService(TIMER3_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER3_COMPC_vect
TIMER_INTERRUPT_HANDLER(TIMER3_COMPC_vect){
	__timer_compareService(TIMER3_COMPAREC);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER4_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER4_OVF_vect){
	__timer_overflowService(TIMER4);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER4_CAPT_vect
TIMER_INTERRUPT_HANDLER(TIMER4_CAPT_vect){
	__timer_captureService(TIMER4);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER4_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER4_COMPA_vect){
	__timer_compareService(TIMER4_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER4_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER4_COMPB_vect){
	__timer_compareService(TIMER4_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER4_COMPC_vect
TIMER_INTERRUPT_HANDLER(TIMER4_COMPC_vect){
	__timer_compareService(TIMER4_COMPAREC);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER5_OVF_vect
TIMER_INTERRUPT_HANDLER(TIMER5_OVF_vect){
	__timer_overflowService(TIMER5);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER5_CAPT_vect
TIMER_INTERRUPT_HANDLER(TIMER5_CAPT_vect){
	__timer_captureService(TIMER5);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER5_COMPA_vect
TIMER_INTERRUPT_HANDLER(TIMER5_COMPA_vect){
	__timer_compareService(TIMER5_COMPAREA);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER5_COMPB_vect
TIMER_INTERRUPT_HANDLER(TIMER5_COMPB_vect){
	__timer_compareService(TIMER5_COMPAREB);
}
#else
# error Missing vector definition
#endif

#ifdef TIMER5_COMPC_vect
TIMER_INTERRUPT_HANDLER(TIMER5_COMPC_vect){
	__timer_compareService(TIMER5_COMPAREC);
}
#else
# error Missing vector definition
#endif
// ----------- End Timer Definition -------------

#endif

// ----------- Define the ADC channels ----------
#define ADC0 ADC_NUMBER_TO_CHANNEL(0)
#define ADC1 ADC_NUMBER_TO_CHANNEL(1)
#define ADC2 ADC_NUMBER_TO_CHANNEL(2)
#define ADC3 ADC_NUMBER_TO_CHANNEL(3)
#define ADC4 ADC_NUMBER_TO_CHANNEL(4)
#define ADC5 ADC_NUMBER_TO_CHANNEL(5)
#define ADC6 ADC_NUMBER_TO_CHANNEL(6)
#define ADC7 ADC_NUMBER_TO_CHANNEL(7)
#define ADC8 ADC_NUMBER_TO_CHANNEL(8)
#define ADC9 ADC_NUMBER_TO_CHANNEL(9)
#define ADC10 ADC_NUMBER_TO_CHANNEL(10)
#define ADC11 ADC_NUMBER_TO_CHANNEL(11)
#define ADC12 ADC_NUMBER_TO_CHANNEL(12)
#define ADC13 ADC_NUMBER_TO_CHANNEL(13)
#define ADC14 ADC_NUMBER_TO_CHANNEL(14)
#define ADC15 ADC_NUMBER_TO_CHANNEL(15)
extern const uint8_t NUM_ADC_CHANNELS;

// ----------- My devices -----------------------
extern SWITCH button;
extern QUADRATURE quad_left;
extern QUADRATURE quad_right;
extern Sharp_GP2D12 ir_dist;
extern SERVO wheel_left;
extern SERVO wheel_right;
extern SERVO_DRIVER wheel_bank;
extern SERVO ir_servo;
extern SERVO_DRIVER ir_servo_bank;
extern SERVO left_elbow;
extern SERVO left_shoulder;
extern SERVO right_elbow;
extern SERVO right_shoulder;
extern SERVO_DRIVER arm_bank;

void initHardware(void);
#endif

// undefine all ports so the user cannot change them directly
#undef PORTA
#undef DDRA
#undef PINA
#undef PORTB
#undef DDRB
#undef PINB
#undef PORTC
#undef DDRC
#undef PINC
#undef PORTD
#undef DDRD
#undef PIND
#undef PORTE
#undef DDRE
#undef PINE
#undef PORTF
#undef DDRF
#undef PINF
#undef PORTG
#undef DDRG
#undef PING
#undef PORTH
#undef DDRH
#undef PINH
#undef PORTJ
#undef DDRJ
#undef PINJ
#undef PORTK
#undef DDRK
#undef PINK
#undef PORTL
#undef DDRL
#undef PINL

// Undefine timer registers to stop users changing them
#undef TCNT0
#undef TCCR0B
#undef OCR0A
#undef OCR0B
#undef TCNT1
#undef TCCR1B
#undef OCR1A
#undef OCR1B
#undef OCR1C
#undef TCNT2
#undef TCCR2B
#undef OCR2A
#undef OCR2B
#undef TCNT3
#undef TCCR3B
#undef OCR3A
#undef OCR3B
#undef OCR3C
#undef TCNT4
#undef TCCR4B
#undef OCR4A
#undef OCR4B
#undef OCR4C
#undef TCNT5
#undef TCCR5B
#undef OCR5A
#undef OCR5B
#undef OCR5C
