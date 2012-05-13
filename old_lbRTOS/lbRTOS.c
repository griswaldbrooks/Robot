#include <FreeRTOS.h>
#include <task.h>
#include <SoR_Utils.h>



void vLEDon(){
	for(;;){

		LED_on();
		vTaskDelay(250/portTICK_RATE_MS);
	}
}

void vLEDblink(){
	for(;;){
		LED_on();
		vTaskDelay(10/portTICK_RATE_MS);
		LED_off();
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

void vLightOn(){
	for(;;){

		PORT_ON(PORTA,0);
		vTaskDelay(250/portTICK_RATE_MS);
	}
}

void vServoOsc(){
	
	for(;;){
		taskENTER_CRITICAL();
		PWM_timer2_Set_H6(127);	
		taskEXIT_CRITICAL();
		
	}
	/*
	for(;;){
		taskENTER_CRITICAL();
		servo(PORTH,6,700);
		taskEXIT_CRITICAL();
		vTaskDelay(250/portTICK_RATE_MS);
	}
	*/
}

void vTaskNULL(){
	
	for(;;){
		vTaskDelay(250/portTICK_RATE_MS);
		
	}
	/*
	for(;;){
		taskENTER_CRITICAL();
		servo(PORTH,6,700);
		taskEXIT_CRITICAL();
		vTaskDelay(250/portTICK_RATE_MS);
	}
	*/
}


int main(void)
{
	//add UART config here
	configure_ports();
	TCCR2B = TIMER2_CLK_64;
	TIMSK2 = _BV(TOIE2); // enable interrupts
 	//TCNT2 = timer2_ovrflow_cnt = 0; // reset counter
	TCNT2 = 0;
	
	
	PWM_Init_timer2_H6(8);
	PWM_timer2_On_H6();
	
	LED_off();
	PWM_timer2_Set_H6(127);	
	//for(;;);	

//	while(!button_pressed());

	xTaskCreate(vLEDon, "LED ON", 10, NULL, 1, NULL);
	xTaskCreate(vLightOn, "Light on", 10, NULL, 1, NULL);
//	xTaskCreate(vServoOsc, "servo go", 100, NULL, 2, NULL);
//	xTaskCreate(vLEDblink, "LED blink", 100, NULL, 1, NULL);
//	xTaskCreate(vTaskNULL, "do NULL", 100, NULL, 1, NULL);
	vTaskStartScheduler();
	for(;;)PORT_ON(PORTA,0);;
	return 0;
}
