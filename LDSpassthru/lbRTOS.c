#include <FreeRTOS.h>
#include <task.h>
//#include <queue.h>
#include <SoR_Utils.h>

#define UNSET 	0x00
#define SET 	0x01
#define SIZE	3500
#define START	0xFA

uint8_t arr[SIZE];
uint8_t full = UNSET;
uint8_t ang_vel_ready = UNSET;
uint8_t ang_vel[2];

void LDSrcv(unsigned char c){	//modified for LDS 06/15/11
		static uint16_t ndx = 0;
		//static uint8_t start_flag = UNSET;
		/*
		if(start_flag){ndx++;}
		if(c == START){ start_flag = SET;}
		if(!ang_vel_ready){
			if(ndx == 2){ang_vel[0] = c;}
			if(ndx == 3){
				ang_vel[1] = c; 
			//	ang_vel_ready = SET;
				for(uint16_t ndx = 0; ndx < 2; ndx++){
				rprintfu08(ang_vel[ndx]);
				}
				rprintfCRLF();
			}
		}
		if(ndx > 3){
			start_flag = UNSET;
			ndx = 0;
		}
		*/
		/*
		if(ndx < SIZE){
			arr[ndx] = c;
			ndx++;
		}
		else{
			full = SET;
		}
		*/
		//c = c & 0b01111111;		//for some reason, every byte has its first bit set to 1
		uart1SendByte(c);
		uart2SendByte(c);
		uart0SendByte(c);
		//rprintf("%c",c);
		//rprintfu08(c);
	
}

void prvSetupHardware(){

	int i, j;

	//add 1.7s delay for potential power issues
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	
	

	uartInit();  // initialize the UART (serial port)
    uartSetBaudRate(0, 115200); // set UARTE speed, for Bluetooth
    uartSetBaudRate(1, 115200); // set UARTD speed, for USB connection, up to 500k, try 115200 if it doesn't work
    uartSetBaudRate(2, 115200); // set UARTH speed
    uartSetBaudRate(3, 115200); // set UARTJ speed, for Blackfin
	//G=Ground, T=Tx (connect to external Rx), R=Rx (connect to external Tx)

	rprintfInit(uart1SendByte);// initialize rprintf system and configure uart1 (USB) for rprintf

	configure_ports(); // configure which ports are analog, digital, etc.
	
	//I2C init
	/*
	i2cInit();  //initialize I2C interface
	i2cSetBitrate(100);  //set I2C transaction bit rate in kHz
	//set local device address and allow
	// response to general call
	i2cSetLocalDeviceAddr(0x02, TRUE); //i2cSetLocalDeviceAddr(LOCAL_ADDR, TRUE);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);
	*/

	
	//UART ISR *** UART ISR ***
	
	uartSetRxHandler(3, &LDSrcv);

	//UART ISR *** UART ISR ***

	LED_on();

	rprintf("\r\nSystem Warmed Up");

	// initialize the timer system
 	init_timer0(TIMER_CLK_1024);
// 	init_timer1(TIMER_CLK_64); // Timer 1 is initialized by FreeRTOS
 	init_timer2(TIMER2_CLK_64);
 	init_timer3(TIMER_CLK_64);
 	init_timer4(TIMER_CLK_64);
 	init_timer5(TIMER_CLK_64);

	a2dInit(); // initialize analog to digital converter (ADC)
	a2dSetPrescaler(ADC_PRESCALE_DIV32); // configure ADC scaling
	a2dSetReference(ADC_REFERENCE_AVCC); // configure ADC reference voltage

	//let system stabelize for X time
	for(i=0;i<16;i++)
		{
		j=a2dConvert8bit(i);//read each ADC once to get it working accurately
		delay_cycles(5000); //keep LED on long enough to see Axon reseting
		rprintf(".");
		}

	LED_off();

	rprintf("Initialization Complete \r\n");

	//reset all timers to zero
	reset_timer0();
	reset_timer1();
	reset_timer2();
	reset_timer3();
	reset_timer4();
	reset_timer5();



	/********PWM Setup***********/
//	prvPWMSetup();

}


void vPassChars(){
	uint8_t fromUser;
	
	uart3SendByte(0x1B);
	uart3SendByte(0x1B);
	uart3SendByte(0x1B);
	//uart3SendByte("SetBaud 256000");
	//uartSetBaudRate(3, 256000);
	//uartSetBaudRate(1, 256000);
	
	for(;;){
		fromUser = uart1GetByte();
		uart3SendByte(fromUser);
		if(fromUser=='G'){
		uart3SendByte("SetBaud 500000\n");
		uartSetBaudRate(3, 500000);
		uart3SendByte(0x1B);
		uart3SendByte(0x1B);
		uart3SendByte(0x1B);
		
		uartSetBaudRate(1, 500000);
		uart1SendByte("Baud Set 5K\n");
		}
	}
	
}

int main(void)
{
	prvSetupHardware();
	
	uint64_t arr[5] = {1,2,3,4,5};
	uint64_t* pp = arr + 1;
	*pp = 6;

	//xTaskCreate(vPassChars, "PassChars", 100, NULL, 1, NULL);
	//vTaskStartScheduler();
	//for(;;){rprintf("DEATH\n");}
	for(;;){

		/*
		if(full){
			for(uint16_t ndx = 0; ndx < SIZE; ndx++){
				rprintf("%c",arr[ndx]);
			}
			for(uint16_t ndx = 0; ndx < SIZE; ndx++){
				rprintfu08(arr[ndx]);
			}

			//for(;;);
		}
		else{
			rprintf("NOT YET\n");
		}
		*/
		/*
		if(ang_vel_ready){
			for(uint16_t ndx = 0; ndx < 2; ndx++){
				rprintf("%c",ang_vel[ndx]);
			}
			ang_vel_ready = UNSET;
		}
		*/
	}
	
	

	
	return 0;
}
