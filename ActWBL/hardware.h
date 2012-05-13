/*
		This file has been auto-generate by WebbotLib tools V1.1
				** DO NOT MODIFY BY HAND **
*/
#ifndef _HARDWARE_H_
#define _HARDWARE_H_
#include "xhardware.h"
// Type of interrupt handler to use for uart interrupts.
// Do not change unless you know what you're doing.
#ifndef UART_INTERRUPT_HANDLER
#define UART_INTERRUPT_HANDLER	ISR
#endif

// ------------------- uart1 -------------------

// Create hardware UART uart1
HW_UART _uart1 = MAKE_UART_BUFFERED(null,null,UCSR1A,UCSR1B,UBRR1L,UBRR1H,UDR1,null,BV(U2X1),D2,D3,&uart1GetByte,&uart1SendByte);

// Create a routine to write bytes to uart1
// You can set rprintf to use it by calling rprintfInit(&uart1SendByte)
MAKE_WRITER(uart1SendByte){
	return uartSendByte(uart1,byte);
}

// Create a routine to read a byte from uart1
// Returns -1 if there was no data
MAKE_READER( uart1GetByte){
	return uartGetByte(uart1);
}

#ifndef USART1_TX_vect
# error Uart1 Tx complete vector undefined
#else
UART_INTERRUPT_HANDLER(USART1_TX_vect){
	uartTransmitService(uart1);
}
#endif

#ifndef USART1_RX_vect
# error Uart1 Rx complete vector undefined
#else
UART_INTERRUPT_HANDLER(USART1_RX_vect){
	uartReceiveService(uart1);
}
#endif

// ------------------- bluetooth -------------------

// Create hardware UART bluetooth
HW_UART _bluetooth = MAKE_UART_BUFFERED(null,null,UCSR0A,UCSR0B,UBRR0L,UBRR0H,UDR0,null,BV(U2X0),E0,E1,&bluetoothGetByte,&bluetoothSendByte);

// Create a routine to write bytes to bluetooth
// You can set rprintf to use it by calling rprintfInit(&bluetoothSendByte)
MAKE_WRITER(bluetoothSendByte){
	return uartSendByte(bluetooth,byte);
}

// Create a routine to read a byte from bluetooth
// Returns -1 if there was no data
MAKE_READER( bluetoothGetByte){
	return uartGetByte(bluetooth);
}

#ifndef USART0_TX_vect
# error Uart0 Tx complete vector undefined
#else
UART_INTERRUPT_HANDLER(USART0_TX_vect){
	uartTransmitService(bluetooth);
}
#endif

#ifndef USART0_RX_vect
# error Uart0 Rx complete vector undefined
#else
UART_INTERRUPT_HANDLER(USART0_RX_vect){
	uartReceiveService(bluetooth);
}
#endif

// ----------- Define the ADC channels ----------
const uint8_t NUM_ADC_CHANNELS = 16;

// ----------- My devices -----------------------
SWITCH button = MAKE_SWITCH(G5);
QUADRATURE quad_left = MAKE_GENERIC_QUADRATURE(K7,K6,true,64);
QUADRATURE quad_right = MAKE_GENERIC_QUADRATURE(K5,K4,false,64);
Sharp_GP2D12 ir_dist = MAKE_Sharp_GP2D12(ADC11);
SERVO wheel_left = MAKE_SERVO(true,C2,1344,900);
SERVO wheel_right = MAKE_SERVO(false,E2,1342,900);
static SERVO_LIST wheel_bank_list[] = {&wheel_left,&wheel_right};
SERVO_DRIVER wheel_bank = MAKE_SERVO_DRIVER(wheel_bank_list);
SERVO ir_servo = MAKE_SERVO(false,H3,1550,880);
static SERVO_LIST ir_servo_bank_list[] = {&ir_servo};
SERVO_DRIVER ir_servo_bank = MAKE_SERVO_DRIVER(ir_servo_bank_list);
SERVO left_elbow = MAKE_SERVO(true,C3,1500,900);
SERVO left_shoulder = MAKE_SERVO(true,C4,1500,900);
SERVO right_elbow = MAKE_SERVO(false,E3,1500,900);
SERVO right_shoulder = MAKE_SERVO(false,E4,1500,900);
static SERVO_LIST arm_bank_list[] = {&left_elbow,&left_shoulder,&right_elbow,&right_shoulder};
SERVO_DRIVER arm_bank = MAKE_SERVO_DRIVER(arm_bank_list);

// ----------- Initialise built in devices ------
void sysInitHardware(void){
	SWITCH_init(&button);
	setErrorLog(&uart1SendByte);
	rprintfInit(&uart1SendByte);
	uartInit(uart1,115200);
}

// ----------- Initialise my added devices ------
void initHardware(void){
	encoderInit(quad_left);
	encoderInit(quad_right);
	distanceInit(ir_dist);
	servosInit(&wheel_bank,TIMER1);
	servosInit(&ir_servo_bank,TIMER3);
	servosInit(&arm_bank,TIMER4);
	uartInit(bluetooth,115200);
}
// ----------- Register the statusLED -----------
void registerLED(void){
	statusLEDregister(B6,false);
}

// ----------- Ports are configured on the fly --
void configure_ports(void){
}

#endif
