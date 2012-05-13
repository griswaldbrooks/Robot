#include <FreeRTOS.h>
#include <task.h>

int i, j;

void vTaskA(void *pvParameters){
	for(;;){
		i++;
		taskYIELD();
			
	}
}
void vTaskB(void *pvParameters){
	for(;;){
		j++;
		taskYIELD();
	}
}



int main(void)
{
	

	xTaskCreate(vTaskB, "B", 10, NULL, 1, NULL);
	xTaskCreate(vTaskA, "A", 10, NULL, 1, NULL);
	vTaskStartScheduler();
	for(;;);
	return 0;
}
