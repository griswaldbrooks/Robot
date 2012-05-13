/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main. c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.  
 * Each task that does not flash an LED maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles an LED.  Should any task contain an error at any time the LED toggle
 * will stop.
 *
 * The LED flash and communications test tasks do not maintain a count.
 */

/*
Changes from V1.2.0
	
	+ Changed the baud rate for the serial test from 19200 to 57600.

Changes from V1.2.3

	+ The integer and comtest tasks are now used when the cooperative scheduler 
	  is being used.  Previously they were only used with the preemptive
	  scheduler.

Changes from V1.2.5

	+ Set the baud rate to 38400.  This has a smaller error percentage with an
	  8MHz clock (according to the manual).

Changes from V2.0.0

	+ Delay periods are now specified using variables and constants of
	  portTickType rather than unsigned long.

Changes from V2.6.1

	+ The IAR and WinAVR AVR ports are now maintained separately.

Changes from V4.0.5

	+ Modified to demonstrate the use of co-routines.

*/

#include <stdlib.h>
#include <string.h>

#ifdef GCC_MEGA_AVR
	/* EEPROM routines used only with the WinAVR compiler. */
	#include <avr/eeprom.h> 
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"


//SoR Include
#include "SoR_Utils.h" //includes all the technical stuff
#include "hardware.c" //declare hardware variables and ports
/*

//#include "CMUcam.c" //not yet written
#include "sensors.c" //sensor libraries for sonar, sharp IR, etc.
#include "misc.c" //includes libraries for various hardware and other useful stuff
#include "axon_DAQ.c" //use the Axon like a data acquisition device
//#include "Blackfin_Axon.c" //files for Blackfin Robot camera
#include "control.c" //your code goes in here
//#include "servo_controller.c" //Axon servo controller
//#include "axon_test.c" //include this is doing a function test for the Axon
//#include "axon_oscope_test.c" //include this is doing a function test for the Axon

#include "a2d.c"
#include "rprintf.c"
#include "uart4.c"
#include "buffer.c"
#include "timer640.c"
*/


#define mainRESET_COUNT_ADDRESS			( ( void * ) 0x50 )

static void vErrorChecks( void *pvParameters );

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay. 
 */
static void prvCheckOtherTasksAreStillRunning( void );

/*
 * Called on boot to increment a count stored in the EEPROM.  This is used to 
 * ensure the CPU does not reset unexpectedly.
 */
static void prvIncrementResetCount( void );

/*
 * The idle hook is used to scheduler co-routines.
 */
void vApplicationIdleHook( void );

void vLEDOn(){
	for(;;){
		LED_on();
	}

}
void vLEDOff(){
	for(;;){
		LED_off();
	}

}

/*-----------------------------------------------------------*/

short main( void )
{

	for(int i=0; i<10000; i++) LED_on();
	
	xTaskCreate(vLEDOn, "LED ON", 100, NULL, 1, NULL);	
	xTaskCreate(vLEDOff, "LED OFF", 100, NULL, 1, NULL);

	vTaskStartScheduler();

	return 0;
}



/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

static void prvIncrementResetCount( void )
{
unsigned char ucCount;

	eeprom_read_block( &ucCount, mainRESET_COUNT_ADDRESS, sizeof( ucCount ) );
	ucCount++;
	eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	vCoRoutineSchedule();
}

