#include "hardware.h"
#include "math.h"


float mean_x;	//acc x axis mean
float var_x = 71;	//acc x axis variance

// Initialise the hardware
void appInitHardware(void) {
	initHardware();
	accelerometerCalibrateX(acc_330, -1047,1012);	//recal'd
	//accelerometerCalibrateY(acc_330, -2187,-1254);	//not recal'd
	//accelerometerCalibrateZ(acc_330, -2203,-1269);	//not recal'd
}

void calc_mean_x(){
	accelerometerRead(acc_330);
	float x = acc_330.accelerometer.x_axis_mG;
	
	for(float iter = 1; iter < 10000; iter++){
		accelerometerRead(acc_330);
		x = acc_330.accelerometer.x_axis_mG;
		mean_x = (1/iter)*x + ((iter - 1)/iter)*mean_x;
		delay_us(25);
	}
	rprintf("Mean: ");
	rprintfFloat(5, mean_x);
	rprintfCRLF();
}

void calc_var_x(){
	accelerometerRead(acc_330);
	float x = acc_330.accelerometer.x_axis_mG;
	
	for(float iter = 1; iter < 5000; iter++){
		x = acc_330.accelerometer.x_axis_mG;
		var_x = (1/iter)*(mean_x - x)*(mean_x - x) + ((iter - 1)/iter)*var_x;
		delay_us(50);
	}
	rprintf("Variance: ");
	rprintfFloat(5, var_x);
	rprintfCRLF();
}

// Initialise the software
TICK_COUNT appInitSoftware(TICK_COUNT loopStart){
	
	delay_ms(1000);	
	rprintf("Entering calc mean\n");
	calc_mean_x();
	rprintf("Exiting calc mean\n");
	rprintf("Entering calc var\n");
	calc_var_x();
	rprintf("Exiting calc var\n");
	delay_ms(1000);	
	
	return 0;
}

int16_t k_filter_x(){

	accelerometerRead(acc_330);
	gyroRead(gyro_300);	
	
	float weight;
	float sig = 100;
	float acc_x = (180*asin(acc_330.accelerometer.x_axis_mG/1000.0))/M_PI;
	float gyro_y;
	float est_x, est_x2 = est_x = acc_x;

	TICK_COUNT start = clockGetus();
	TICK_COUNT end;
	float duration;

	for(float iter = 0; iter < 100; iter++){
		gyro_y = gyro_300.gyro.y_axis_degrees_per_second;
		sig += 0.001;
		end = clockGetus();
		duration = end - start;
		est_x += fma(duration,0.000001,0)*gyro_y;

		accelerometerRead(acc_330);
		start = clockGetus();
		gyroRead(gyro_300);	
		acc_x = (180*asin(acc_330.accelerometer.x_axis_mG/1000.0))/M_PI;
		weight = sig/(sig + var_x);
		est_x2 += weight*(acc_x - est_x2);
		est_x += weight*(acc_x - est_x);
		sig = (1 - weight)*sig;
		}
//	rprintf("%d\t%d",(int16_t)est_x,(int16_t)est_x2);
	return est_x;

}

int16_t k_filter_ang(){

	accelerometerRead(acc_330);
	gyroRead(gyro_300);	
	
	float weight, h;
	float sig = 100;
	float acc_x = (180*asin(acc_330.accelerometer.x_axis_mG/1000.0))/M_PI;
	float gyro_y = gyro_300.gyro.y_axis_degrees_per_second;
	float est_ang = acc_x;
	//float est_vel = gyro_y;

	TICK_COUNT start = clockGetus();
	TICK_COUNT end;
	float duration;

	for(float iter = 0; iter < 100; iter++){
	//while(1){
		gyro_y = gyro_300.gyro.y_axis_degrees_per_second;
		sig += 0.001;
		end = clockGetus();
		duration = end - start;
		est_ang += fma(duration,0.000001,0)*gyro_y;

		accelerometerRead(acc_330);
		start = clockGetus();
		gyroRead(gyro_300);	
		acc_x = (180*asin(acc_330.accelerometer.x_axis_mG/1000.0))/M_PI;
		//h = (180/M_PI)/(sqrt(1 - square(acc_330.accelerometer.x_axis_mG/1000.0)));
		h = 1;
		weight = h*sig/(square(h)*sig + var_x);
		est_ang += weight*(acc_x - est_ang);
		sig = (1 - h*weight)*sig;
		
		//rprintf("%d\n",(int16_t)est_ang);
		//delay_us(5000);
	}

	return est_ang;

}

int16_t max_z = -32000;
int16_t min_z = 32000;
int16_t new_est_x = 0;
int16_t old_est_x = 0;
int16_t sum = 0;
char adj;
float iter = 0;

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

	// -------- Start ADXL335 3 Axis Accelerometer-------
	// Read the ADXL335 3 Axis Accelerometer and store results
//	accelerometerRead(acc_330);
//	gyroRead(gyro_300);
	// The x axis value is stored in acc_330.accelerometer.x_axis_mG;
	// The y axis value is stored in acc_330.accelerometer.y_axis_mG;
	// The z axis value is stored in acc_330.accelerometer.z_axis_mG;
	// Each value can be printed using %d eg rprintf("x=%d",acc_330.accelerometer.x_axis_mG);
	// or all values can be dumped out using
//	ACCEL_TYPE x = acc_330.accelerometer.x_axis_mG;
//	int16_t x = acc_330.accelerometer.x_axis_mG;
//	if(z > max_z){max_z = z;}
//	if(z < min_z){min_z = z;}
	//est_x = (-k_filter_x()/20);
	//mean_x = (1/iter)*est_x + ((iter - 1)/iter)*mean_x;
	//iter++;

//	rprintf("MAX Z: %d\tMIN Z: %d",max_z, min_z);
//	rprintf("%d",a2dConvert8bit(adc0));
	//rprintf("%d",x/10 + 50);
	//delay_ms(10);
	//sum = k_filter_x();
	rprintf("%d",k_filter_ang());
	//rprintfFloat(5, (float)k_filter_x());
	adj = uart1GetByte();
	if(adj == 'p'){ var_x+= 10;}
	else if(adj == 'l'){ var_x -= 10;}
//	accelerometerDump(acc_330);
	rprintfCRLF();
	// -------- End   ADXL335 3 Axis Accelerometer-------

	return 0;
}
