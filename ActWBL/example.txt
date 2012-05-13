#include "hardware.h"

// Initialise the hardware
void appInitHardware(void) {
	initHardware();
}

// Initialise the software
TICK_COUNT appInitSoftware(TICK_COUNT loopStart){
	return 0;
}

// This is the main loop
TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart) {

	// -------- Start Switch-------
	// Switch - see switch.h
	
	// To test if it is pressed then
	if(SWITCH_pressed(&button)){
		// pressed
	}
	
	// To test if it is released then
	if(SWITCH_released(&button)){
		// released
	}
	// -------- End   Switch-------

	// -------- Start Quadrature-------
	// Read the Quadrature and store the result in quad_left.encoder.value
	encoderRead(quad_left);
	
	// The value can be printed using %d eg rprintf("Ticks=%d",quad_left.encoder.value);
	// or dumped using:
	rprintf("quad_left: ");
	encoderDump(quad_left);
	rprintfCRLF();
	// -------- End   Quadrature-------

	// -------- Start Quadrature-------
	// Read the Quadrature and store the result in quad_right.encoder.value
	encoderRead(quad_right);
	
	// The value can be printed using %d eg rprintf("Ticks=%d",quad_right.encoder.value);
	// or dumped using:
	rprintf("quad_right: ");
	encoderDump(quad_right);
	rprintfCRLF();
	// -------- End   Quadrature-------

	// -------- Start Sharp GP2-------
	// Read the Sharp GP2 and store the result in ir_dist.distance.cm
	distanceRead(ir_dist);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_dist.distance.cm);
	// or dumped using:
	rprintf("ir_dist: ");
	distanceDump(ir_dist);
	rprintfCRLF();
	// -------- End   Sharp GP2-------

	// -------- Start Actuators -------
	// To control your.motors/servos then see actuators.h in the manual	// To retrieve the required speed of wheel_left use:
	// DRIVE_SPEED speed=act_getSpeed(wheel_left);
	// To set the required speed of wheel_left use:
	// act_setSpeed(wheel_left,speed);
	// This example will move the motors back and forth using the loopStart time:
	TICK_COUNT ms = loopStart / 1000;		// Get current time in ms
	int16_t now = ms % (TICK_COUNT)10000; 	// 10 sec for a full swing
	if(now >= (int16_t)5000){				// Goes from 0ms...5000ms
		now = (int16_t)10000 - now;			// then 5000ms...0ms
	}
	// Map it into DRIVE_SPEED range
	DRIVE_SPEED speed = interpolate(now, 0, 5000, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
	// Set speed for all motors/servos
	act_setSpeed(&wheel_left,speed);
	act_setSpeed(&wheel_right,speed);
	act_setSpeed(&ir_servo,speed);
	act_setSpeed(&left_elbow,speed);
	act_setSpeed(&left_shoulder,speed);
	act_setSpeed(&right_elbow,speed);
	act_setSpeed(&right_shoulder,speed);
	// -------- End   Actuators -------

	return 0;
}
