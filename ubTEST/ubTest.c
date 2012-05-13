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

	// -------- Start Switch/Button-------
	// Switch/Button - see switch.h
	
	// To test if it is pressed then
	if(SWITCH_pressed(&button)){
		// pressed
	}
	
	// To test if it is released then
	if(SWITCH_released(&button)){
		// released
	}
	// -------- End   Switch/Button-------

	// -------- Start Marquee-------
	// Marquee - see 'segled.h'
	// Before using the Marquee you need to redirect rprintf to write to it
	// This can be done using
	Writer old = rprintfInit(marqueeGetWriter(&marquee));
	
	// All rprintf output will then be sent to the marquee but will not
	// display until an end-of-line eg "\n" has been sent. Example:-
	// rprintf("Hello World\n");
	
	// If the endDelay is non-zero then the marquee will scroll
	// forever or until you call: marqueeStop(&marquee);
	
	// If the endDelay is zero then the marquee will stop once
	// the entire line has been shown ('one-shot' mode)
	
	// In 'one-shot' mode then you may want to make sure that
	// a previous line has finished before you display a second line.
	// This can be done as follows:-
	marqueeSetEndDelay(&marquee,0); // Make sure we are in one-shot mode
	if(marqueeIsActive(&marquee)==FALSE){
	     if(loopCount==1){
	     	rprintf("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n");
	     }else{
			rprintf("Loop=%u\n",(unsigned)loopCount); // Put the loop count
	     }
	}
	
	// Restore rprintf back to its previous location
	rprintfInit(old);
	// -------- End   Marquee-------

	// -------- Start Sharp GP2-------
	// Read the Sharp GP2 and store the result in ir_0.distance.cm
	distanceRead(ir_0);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_0.distance.cm);
	// or dumped using:
	rprintf("ir_0: ");
	distanceDump(ir_0);
	rprintfCRLF();
	// -------- End   Sharp GP2-------

	// -------- Start Sharp GP2-------
	// Read the Sharp GP2 and store the result in ir_1.distance.cm
	distanceRead(ir_1);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_1.distance.cm);
	// or dumped using:
	rprintf("ir_1: ");
	distanceDump(ir_1);
	rprintfCRLF();
	// -------- End   Sharp GP2-------

	// -------- Start Sharp GP2-------
	// Read the Sharp GP2 and store the result in ir_2.distance.cm
	distanceRead(ir_2);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_2.distance.cm);
	// or dumped using:
	rprintf("ir_2: ");
	distanceDump(ir_2);
	rprintfCRLF();
	// -------- End   Sharp GP2-------

	// -------- Start Sharp GP2-------
	// Read the Sharp GP2 and store the result in ir_3.distance.cm
	distanceRead(ir_3);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_3.distance.cm);
	// or dumped using:
	rprintf("ir_3: ");
	distanceDump(ir_3);
	rprintfCRLF();
	// -------- End   Sharp GP2-------

	// -------- Start Generic I2C Device-------
	// Read 3 register values starting at register 1 into response
	uint8_t lbRTOS_response[3];
	if(i2cMasterReadRegisters(&lbRTOS.i2cInfo, 1, sizeof(lbRTOS_response), lbRTOS_response)){
		// We have successfully read the data into 'lbRTOS_response'
	}else{
		rprintf("Failed reading lbRTOS\n");
	}
	// -------- End   Generic I2C Device-------

	return 0;
}
