//#define UART_TX_BUFFER_SIZE 80
//#define UART_RX_BUFFER_SIZE 200

#include <stdlib.h> 
#include <assert.h>
#include "hardware.h"
#include <math.h>

#define NDEBUG

//Uniform Grid dimensions
#define N		26	//26
#define HALF_N	13

#define SET				0x01
#define UNSET			0x00
#define MAX_F			0xFF	//max A* f value for a freespace point
#define NUM_IR_READS	10
#define GRID_UNIT_RES	10

const float HALF_PI = M_PI/2;
const float QTR_PI = M_PI/4;

typedef struct {
	int16_t heading;
	int16_t sen_heading;
	int16_t sen_vel;
	float dDis;			//sensed robot distance vector magnitude
	float x;
	float y;
} R_POSE;

typedef struct{
	uint8_t x;
	uint8_t y;
} GOAL;

typedef struct {
	float magnitude;
	int16_t angle;
	float x;
	float y;
	uint8_t valid;
} RCT_PT; //recent point received by an ir sensor

struct ERROR_VECT{
	float dx;
	float dy;
	RCT_PT* rct_pt;
	struct ERROR_VECT* next;
	struct ERROR_VECT* prev;
};

struct ERROR_LIST{
	struct ERROR_VECT* head;
	struct ERROR_VECT* tail;
	uint8_t size;
};

struct MAP_PT{
	uint8_t occ_flag;
	void* point;
};

struct OCC_PT{
	int8_t k;				//confidence of the point, out of 100
	//struct OCC_PT* next; 	//for updating all of the points in the list of points
	//struct OCC_PT* prev; 	//for updating all of the points in the list of points
};

struct FRE_PT{
	uint8_t f;				//manhattan distance to goal + manhattan distance to robot
	struct FRE_PT* backptr;	//for A*
};

struct MAP_PT  g_map[N][N];
//struct ERROR_VECT* err_list[NUM_OF_IR][ELEMENTS_IN_ERR_WINDOW];
RCT_PT recent_ptset[5];
struct ERROR_LIST err_list;

struct FRE_PT* open_list;
struct FRE_PT* clos_list;

R_POSE robot;
GOAL start;



//-----------------------------**&&**

float deg_to_rad(const int16_t deg){
	return (deg * M_PI)/180;
}

float rad_to_deg(const int16_t rad){
	return (rad * 180)/M_PI;
}

struct OCC_PT* init_occ_pt(){
	struct OCC_PT* pt = malloc(sizeof(struct OCC_PT));
	pt->k = 5; 	//start with a low initial confidence
	//pt->next = NULL;
	//pt->prev = NULL;
	return pt;
	//rprintf(" %d ", pt->k);
}

struct FRE_PT* init_fre_pt(){
	struct FRE_PT* pt = malloc(sizeof(struct FRE_PT));
	pt->f = MAX_F;	//distance to start + distance to goal
	pt->backptr = NULL;
	return pt;
}


struct ERROR_VECT* init_err_vect(uint8_t dx, uint8_t dy, RCT_PT* rct_pt,
								 struct ERROR_VECT* next, struct ERROR_VECT* prev){
	struct ERROR_VECT* evct = malloc(sizeof(struct ERROR_VECT));
	if(evct == NULL){rprintf("NULL");}
//	else{rprintf("NOT NULL");}
	evct->dx = dx;
	evct->dy = dy;
	evct->next = next;
	evct->prev = prev;
	evct->rct_pt = rct_pt;
	return evct;
}

void delete_err_vect(struct ERROR_VECT* evct){
	evct->prev->next = evct->next;
	evct->next->prev = evct->prev;
	free(evct);
}

void delete_occ_pt(struct OCC_PT* pt){
	//pt->prev->next = pt->next;
	//pt->next->prev = pt->prev;
	free(pt);

}

void delete_fre_pt(struct FRE_PT* pt){
	//free(pt->backptr);
	free(pt);

}

void init_map_pt(struct MAP_PT* mpt, uint8_t occ_flag){
		if(occ_flag){	//occlusion
			mpt->point = (void*)init_occ_pt();
			mpt->occ_flag = SET;
		}	
		else{			//free point
			mpt->point = (void*)init_fre_pt();
			mpt->occ_flag = UNSET;
		}
}

void insert_map_pt(uint8_t x, uint8_t y, uint8_t occ_flag){
	
	if((x > N) || (x < 0)){rprintf(" |PROBLEM X: %d| ",x);}
	assert(!(x > N));
	assert(!(x < 0));
	if((y > N) || (y < 0)){rprintf(" |PROBLEM Y: %d| ",y);}
	assert(!(y > N));
	assert(!(y < 0));
	
	if(g_map[x][y].occ_flag && occ_flag){ 	//there is an occlusion at this point and another occlusion was detected there
		((struct OCC_PT*)g_map[x][y].point)->k += 5;
		if(((struct OCC_PT*)g_map[x][y].point)->k > 100){
			((struct OCC_PT*)g_map[x][y].point)->k = 100;
		}
		return;
	}
	else if(g_map[x][y].occ_flag){ 	//there is an occlusion at this point
		delete_occ_pt((struct OCC_PT*)g_map[x][y].point);
	}
	else{				//this point is free
		delete_fre_pt((struct FRE_PT*)g_map[x][y].point);
	}
	init_map_pt(&(g_map[x][y]), occ_flag);
}

void init_map(){
	uint8_t unset = UNSET;
	for(uint8_t i=0; i<N; i++){
		for(uint8_t j=0; j<N; j++){
			init_map_pt(&g_map[i][j], unset); //create default freespace points
		}
	}
}

void init_rct_set(){
	for(uint8_t itr = 0; itr < 5; itr++){
		recent_ptset[itr].magnitude = 0;
		recent_ptset[itr].valid = UNSET;
	}
}

void init_robot(){
	robot.heading = 0;
	robot.sen_heading = 0;
	robot.sen_vel = 0;
	robot.dDis = 0;
	robot.x = HALF_N;
	robot.y = HALF_N;

}

void init_error_list(){
	err_list.head = init_err_vect(0, 0, NULL, NULL, NULL);
	err_list.tail = init_err_vect(0, 0, NULL, NULL, err_list.head);
	err_list.head->next = err_list.tail;
	err_list.size = 0;
}

void insert_after_err(struct ERROR_VECT* ins_pt, uint8_t dx, uint8_t dy, RCT_PT* rct_pt){
	struct ERROR_VECT* temp = ins_pt->next;
	ins_pt->next = init_err_vect(dx, dy, rct_pt, temp, ins_pt);
	//ins_pt->next->prev = ins_pt;
	//ins_pt->next->next = temp;
	temp->prev = ins_pt->next;
	err_list.size++;
}

void push_back_err(uint8_t dx, uint8_t dy, RCT_PT* rct_pt){
//		rprintf("PUSH START:.");
		insert_after_err(err_list.tail->prev, dx, dy, rct_pt);
//		rprintf(".:PUSH DONE\n");
}

struct ERROR_VECT ret_bad_err_vect(){
	struct ERROR_VECT bad_err_vect;
	bad_err_vect.dx = 0xFFFFFFFF;
	bad_err_vect.dy = 0xFFFFFFFF;
	bad_err_vect.next = NULL;
	bad_err_vect.prev = NULL;
	bad_err_vect.rct_pt = NULL;
	return bad_err_vect;
}

struct ERROR_VECT pop_back_err(){
	if(err_list.size == 0){return ret_bad_err_vect();} //list is empty
	struct ERROR_VECT retval = *(err_list.tail->prev);
	delete_err_vect(err_list.tail->prev);
	err_list.size--;
	return retval;
	
}

void clear_err_list(){
	struct ERROR_VECT grab;
	while(err_list.size != 0){ //while list is not empty
		grab = pop_back_err();
	//	rprintf(" dx:");
	//	rprintfFloat(5, grab.dx);
	//	rprintf(" ");
	//	rprintf(" dy:");
	//	rprintfFloat(5, grab.dy);
	//	rprintf(" ");
	//	rprintf("size: %d | ",err_list.size);
	}
}

void update_pts(){
	for(int i=0; i<N; i++){
		for(int j=0; j<N; j++){
			if(g_map[i][j].occ_flag){ 
				
				((struct OCC_PT*)g_map[i][j].point)->k--; //decrement the confidence
				if(((struct OCC_PT*)g_map[i][j].point)->k < 0){	//replace with free point
					insert_map_pt(i, j, UNSET);
				}
				if(g_map[i][j].occ_flag){	//if there's still a point here
					if(robot.dDis > GRID_UNIT_RES){	//if the robot has moved enough to make a map point shift
						uint8_t i_new = -(robot.dDis * cos(deg_to_rad(robot.heading))) + i;
						uint8_t j_new = -(robot.dDis * sin(deg_to_rad(robot.heading))) + j;

						if((i_new <= N) && (j_new <= N)){	//if the new position is within the map
							insert_map_pt(i_new, j_new, SET);
							//update confidence
							((struct OCC_PT*)g_map[i_new][j_new].point)->k = ((struct OCC_PT*)g_map[i][j].point)->k;
						}
						insert_map_pt(i, j, UNSET);	//get rid of old map point
					}
				}
	
			}
		}
	}
}


void print_map(){
	for(int n=0; n<N; n++){
		rprintf("_");
		}
	rprintf("\n");
	for(int i=0; i<N; i++){
		for(int j=0; j<N; j++){
			if((i == robot.x)&&(j == robot.y)){ rprintf("R");}
			else if(g_map[i][j].occ_flag){
				if(((struct OCC_PT*)g_map[i][j].point)->k < 25){rprintf("o");}
				else if(((struct OCC_PT*)g_map[i][j].point)->k < 50){rprintf("O");}
				else if(((struct OCC_PT*)g_map[i][j].point)->k < 75){rprintf("@");}
				else{rprintf("#");}
			 }
			else{rprintf(".");}
		}
		rprintf("|\n");
	}
	for(int n=0; n<N; n++){
		rprintf("_");
	}
	rprintf("\v");
}


char num2char(char c){
	
	if(c <10){
		c += 48;
	}
	else if((c >= 10) && (c <= 16)){
		c += 55;
	}
	return c;
}

char char2hex(char c){
	
	if((c > 47) && (c <58)){
		c -= 48;
	}
	else if((c > 64) && (c <71)){
		c -= 55;
	}
	return c;
}

void lbRcv(unsigned char c){
	//posts sensed heading and velocity data to global variable
	static char lf_flag; //checks if line feed ("\n") character has been sent
	static char h_iter; //count iterator for sensed heading
	static char v_iter; //count iterator for sensed velocity
	static char d_iter;	//count iterator for differential distance 
	static char h_flag;
	static char v_flag;
	static char d_flag;
	static signed int h_rough = 0;  //store heading
	static signed int v_rough = 0;  //store velocity
	static signed int d_rough = 0;  //store differential distance 
		//rprintf("%c",c);
		if(1){//c != -1
		//if the data isn't whitespace (0xff), post it
			//rprintf("%c",c);

			if(c == 0x0A){lf_flag = SET;} //line feed detected, the character will be a 'H','V', or 'S'
	
			else if((lf_flag) && (c == 'H')){ //set heading flag
				h_flag = SET;
				h_iter = 0;
				lf_flag = UNSET;
				return;
			} 
			else if((lf_flag) && (c == 'V')){ //set velocity flag
				v_flag = SET;
				v_iter = 0;
				lf_flag = UNSET;
			//	rprintf("%c",c);
				return;
			}
			else if((lf_flag) && (c == 'S')){ //set differential distance  flag
				d_flag = SET;
				d_iter = 0;
				lf_flag = UNSET;
				return;
			}
			else if(h_flag){
				h_rough = (char2hex(c) | (h_rough << 4));	//store then increment	
				h_iter++;
		//		rprintfu08(c);
		//		rprintf(" ITER:%d|",h_iter);
		//		rprintfu16(h_rough);
		//		rprintf("\t");
		//		rprintf("%c",c);
			}	
			else if(v_flag){
				v_rough = (char2hex(c) | (v_rough << 4));	//store then increment	
				v_iter++;
			//	rprintf("%c",c);
			}
			else if(d_flag){
				d_rough = (char2hex(c) | (d_rough << 4));	//store then increment	
				d_iter++;
			//	rprintf("%c",c);
			}

			if(h_iter == 4){
				robot.sen_heading = h_rough;
				h_flag = UNSET;
				h_iter = 0;
		//		rprintf("H: %d",h_rough);
		//		rprintfNum(2, 16, 1, ' ', h_rough);
		//		rprintfu16(h_rough);
		//		rprintf("\n");
				h_rough = 0;
			}
			else if(v_iter == 4){
				robot.sen_vel = v_rough;
				v_flag = UNSET;
				v_iter = 0;
		//		rprintf("V: ");
		//		rprintfu16(v_rough);
		//		rprintf("\n");
				v_rough = 0;
			}
			else if(d_iter == 4){
				robot.dDis += (float)d_rough/10;	//differential distance is added to current differential, retrieving function will reset to zero
				d_flag = UNSET;			//divided by 10 to reintroduce the decimals
				d_iter = 0;
		//		rprintf("D: ");
		//		rprintfu16(d_rough);
		//		rprintf("\n");
				d_rough = 0;
			}
		
		}

		else{rprintf("WR\n");}

}

// Initialise the hardware
void appInitHardware(void) {
	initHardware();


}
// Initialise the software
TICK_COUNT appInitSoftware(TICK_COUNT loopStart){
	init_robot();
	init_error_list();
	init_map();
	init_rct_set();
	uartAttach(UART3, &lbRcv);
	return 0;
}

void send_angle(int16_t angle){
//send commanded angle
	uint8_t r1 = 0;
	uint8_t r2 = 0;
	uint8_t r3 = 0;
	uint8_t r4 = 0;

	uartSendByte(UART3, 'R');

	r1 = num2char(0x0F & (uint8_t)angle);
	r2 = num2char(0x0F & ((uint8_t)(angle >> 4)) );
	r3 = num2char(0x0F & ((uint8_t)(angle >> 8)) );
	r4 = num2char(0x0F & ((uint8_t)(angle >> 12)) );
	/*
	uartSendByte(UART1, r4);
	uartSendByte(UART1, r3);
	uartSendByte(UART1, r2);
	uartSendByte(UART1, r1);
	uartSendByte(UART1, '\r');
	uartSendByte(UART1, '\n'); //line feed
	*/
	uartSendByte(UART3, r4);
	uartSendByte(UART3, r3);
	uartSendByte(UART3, r2);
	uartSendByte(UART3, r1);
	uartSendByte(UART3, '\n'); //line feed
}


void push_adj_open(struct FRE_PT* current_ptr){
	for(uint8_t itr = 0; itr < 4; itr++){
		//set_back_ptr(
		//set_f_score(
		
	}
}

struct FRE_PT* A_star(){	//Searches from the goal to the robot and returns the FRE_PT in front of the robot.
							//The robot will then have to trace the back pointers to the goal
	//initialize the first node on the open list
	open_list = (struct FRE_PT*)g_map[start.x][start.y].point;

	//pop current node onto open list (goal)
	push_adj_open(open_list);
	//check adjacent nodes
	//push adj nodes onto open list
		//setting their back pointers to the current node
		//assign their f scores
	//pop current node off of open list and onto closed list
	//evaluate the node with the lowest f score on the list
	//end when the current node is if front of the robot

}

void update_rpt_xyang(){
	recent_ptset[0].angle = -90 + robot.heading;
	recent_ptset[1].angle = robot.heading;
	recent_ptset[2].angle = 90 + robot.heading;
	recent_ptset[3].angle = 45 + robot.heading;
	recent_ptset[4].angle = -45 + robot.heading;

	for(uint8_t itr = 0; itr < 5; itr++){
		recent_ptset[itr].x = recent_ptset[itr].magnitude * cos(deg_to_rad(recent_ptset[itr].angle)) + robot.x;
	//	rprintf("rxu ");
	}

	for(uint8_t itr = 0; itr < 5; itr++){
		recent_ptset[itr].y = recent_ptset[itr].magnitude * sin(deg_to_rad(recent_ptset[itr].angle)) + robot.y;
	//	rprintf("ryu ");
	}
	//rprintfCRLF();
}

void read_dis_arr(){
	float var = 50;
	float weight;
	float sig = 50;

	init_rct_set();

	distanceRead(ir_0);
	distanceRead(ir_1);
	distanceRead(ir_2);
	distanceRead(ir_3);
	distanceRead(ir_4);

	recent_ptset[0].magnitude = (float)ir_0.distance.cm;
	recent_ptset[1].magnitude = (float)ir_1.distance.cm;
	recent_ptset[2].magnitude = (float)ir_2.distance.cm;
	recent_ptset[3].magnitude = (float)ir_3.distance.cm;
	recent_ptset[4].magnitude = (float)ir_4.distance.cm;
	
	//Kalman Filter
	for(float iter = 0; iter < 5; iter++){
		distanceRead(ir_0);
		distanceRead(ir_1);
		distanceRead(ir_2);
		distanceRead(ir_3);
		distanceRead(ir_4);
		
		weight = sig/(sig + var);

		recent_ptset[0].magnitude += weight*((float)ir_0.distance.cm - recent_ptset[0].magnitude);
		recent_ptset[1].magnitude += weight*((float)ir_1.distance.cm - recent_ptset[1].magnitude);
		recent_ptset[2].magnitude += weight*((float)ir_2.distance.cm - recent_ptset[2].magnitude);
		recent_ptset[3].magnitude += weight*((float)ir_3.distance.cm - recent_ptset[3].magnitude);
		recent_ptset[4].magnitude += weight*((float)ir_4.distance.cm - recent_ptset[4].magnitude);

		sig = (1 - weight)*(sig + 0.0001);
	}
	//scale to proper grid resolution
	for(uint8_t iter = 0; iter < 5; iter++){
		recent_ptset[iter].magnitude = recent_ptset[iter].magnitude/GRID_UNIT_RES;
	}
	
	for(uint8_t itr = 0; itr < 5; itr++){
		if(recent_ptset[itr].magnitude < HALF_N){ recent_ptset[itr].valid = SET;
	//		rprintf(" IR%d: ",itr);
	//		rprintfFloat(5, recent_ptset[itr].magnitude);
		}
		else{ recent_ptset[itr].valid = UNSET;}
	}

	update_rpt_xyang();


}

void update_transl(){
	robot.x += robot.dDis * cos(deg_to_rad(robot.heading));
	robot.y += robot.dDis * sin(deg_to_rad(robot.heading));
}
void update_heading(){
	robot.heading = robot.sen_heading;
}



void chk_srndng(uint8_t rpt_ndx){
	//check the 3x3 box around the recent point
	for(uint8_t i = recent_ptset[rpt_ndx].x - 1; i < recent_ptset[rpt_ndx].x + 2; i++){
		for(uint8_t j = recent_ptset[rpt_ndx].y - 1; j < recent_ptset[rpt_ndx].y + 2; j++){
			if((i>= 0 && i <= N) && (j>=0 && j <= N)){
				if(g_map[i][j].occ_flag == SET){
					//rprintf("i: %d\tj: %d\n",i,j);
					push_back_err(recent_ptset[rpt_ndx].x - i, recent_ptset[rpt_ndx].y - j, &recent_ptset[rpt_ndx]);
				}
			}
		}
	}
	//rprintf("END CHK S\n");
}

float calc_heading(){
	float sum_weight = 0;
	float sum_dE_theta_w = 0;

	float e_x;
	float e_y;
	
	float e_r;
	float e_theta;	//in degrees
	
	float dE_r;
	float dE_theta;	//in degrees
	
	float weight;
	float dE_theta_w;

	for(struct ERROR_VECT* itr = err_list.head->next; itr != err_list.tail; itr = itr->next){

		e_x = itr->rct_pt->x + itr->dx;
		e_y = itr->rct_pt->y + itr->dy;
	
		e_r = sqrt(square(e_x) + square(e_y));
		e_theta = rad_to_deg(atan2(e_x, e_y));
	
		dE_r = itr->rct_pt->magnitude - e_r;
		dE_theta = itr->rct_pt->angle - e_theta;
	
		weight = 1/(fabs(dE_r));
		dE_theta_w = (dE_theta)*(weight);

		sum_weight += weight;
		sum_dE_theta_w += dE_theta_w;
	}
	return sum_dE_theta_w/sum_weight;

}

void plot_rpts(){
	for(uint8_t rpt_ndx = 0; rpt_ndx < 5; rpt_ndx++){
		if(recent_ptset[rpt_ndx].valid){
	//		rprintf(" rpt.x: %d rpt.y: %d rpt.m: %d rpt.a: %d|",
	//				(uint8_t)recent_ptset[rpt_ndx].x,(uint8_t)recent_ptset[rpt_ndx].y,
	//				(int16_t)recent_ptset[rpt_ndx].magnitude, recent_ptset[rpt_ndx].angle);
			insert_map_pt(((uint8_t)(recent_ptset[rpt_ndx].x)), ((uint8_t)(recent_ptset[rpt_ndx].y)), SET);
			//insert_map_pt(5, 6, SET);
		}
	}
//	rprintf("\nPLOT END\n");
}


void lclz_plot(){
	int16_t lclzd_heading = 0;
	float calc_var = 18;
	float process_var = 10;
	float gain;
	read_dis_arr();
	
	robot.heading = robot.sen_heading;

	for(uint8_t iter = 0; iter < 10; iter++){
		
		process_var += 0.001;

		for(uint8_t rpt_ndx = 0; rpt_ndx < 5; rpt_ndx++){
			if(recent_ptset[rpt_ndx].valid){chk_srndng(rpt_ndx);}
		}
		lclzd_heading = calc_heading();

		gain = process_var/(process_var + calc_var);
		robot.heading += gain*(lclzd_heading - robot.heading);
		process_var = (1 - gain)*process_var;
		
		clear_err_list();	
		update_rpt_xyang();
	}
	plot_rpts();
	update_pts();
	
	rprintf("%d\n", robot.heading);

	//print_map();
	if(robot.dDis != 0){
	//	rprintf("dDis: ");
	//	rprintfFloat(5,robot.dDis);
	}
	robot.dDis = 0;

	

}



int x = 5;
int y;
//uint8_t lbRTOS_response[3];
long counter = 0;
int16_t cmd_r = 0;



// This is the main loop
TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart) {
	//rprintf("\t\t---------------------COUNTER: %d\n", counter);
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
/* FIGURE EIGHT TEST
	
	for(uint8_t i = 0; i < 90; i++){
		send_angle(cmd_r);
		delay_ms(62);
		rprintf("lb: %d\n",robot.sen_heading);
		cmd_r++;
	}
	for(uint8_t i = 0; i < 180; i++){
		send_angle(cmd_r);
		delay_ms(62);
		rprintf("lb: %d\n",robot.sen_heading);
		cmd_r--;
	}
	for(uint8_t i = 0; i < 90; i++){
		send_angle(cmd_r);
		delay_ms(62);
		rprintf("lb: %d\n",robot.sen_heading);
		cmd_r++;
	}
	
*/
//	rprintf("lb vel: %d\n",robot.sen_vel);
	//rprintf("MAIN LOOP\n");
//	lclz_plot();
	//ir_filter();
	send_angle(5);
	// -------- Start Sharp GP2------- // -90 degrees
	// Read the Sharp GP2 and store the result in ir_0.distance.cm
	//distanceRead(ir_0);
	//rprintf("P %d\n",robot.sen_heading);	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_0.distance.cm);
	// or dumped using:
	//rprintf("ir_0: ");
	//distanceDump(ir_0);
	//rprintfCRLF();
	//x = (int)( (double)(ir_0.distance.cm / 2) * cos((-M_PI / 2) + robot.sen_heading) ) + 24;
	//y = (int)( (double)(ir_0.distance.cm / 2) * sin((-M_PI / 2) + robot.sen_heading) ) + 24;
	//g_map[x][y] = 1;
	//rprintf("%d,%d,1\n", x, y); //x, y,, intensity
	// -------- End   Sharp GP2-------

	// -------- Start Sharp GP2------- // 0 degrees
	// Read the Sharp GP2 and store the result in ir_1.distance.cm
	//distanceRead(ir_1);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_1.distance.cm);
	// or dumped using:
	//rprintf("ir_1: ");
	//distanceDump(ir_1);
	//rprintfCRLF();
	//x = (int)( (double)(ir_1.distance.cm / 2) * cos(0 + robot.sen_heading) ) + 24;
	//y = (int)( (double)(ir_0.distance.cm / 2) * sin(0 + robot.sen_heading) ) + 24;
	//g_map[x][y] = 1;
	//rprintf("%d,%d,1\n", x, y);//x, y,, intensity
	// -------- End   Sharp GP2-------




	// -------- Start Sharp GP2------- // 90 degrees
	// Read the Sharp GP2 and store the result in ir_2.distance.cm
	//distanceRead(ir_2);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_2.distance.cm);
	// or dumped using:
	//rprintf("ir_2: ");
	//distanceDump(ir_2);
	//rprintfCRLF();
	//x = (int)( (double)(ir_2.distance.cm / 2) * cos((M_PI / 2) + robot.sen_heading) ) + 24;
	//y = (int)( (double)(ir_2.distance.cm / 2) * sin((M_PI / 2) + robot.sen_heading) ) + 24;
	//g_map[x][y] = 1;
	//rprintf("%d,%d,1\n", x, y);//x, y,, intensity
	// -------- End   Sharp GP2-------
/**/
	// -------- Start Sharp GP2------- //45 degrees
	// Read the Sharp GP2 and store the result in ir_3.distance.cm
	//distanceRead(ir_3);
	
	// The value can be printed using %u eg rprintf("Distance=%u",ir_3.distance.cm);
	// or dumped using:
	//rprintf("\tir_3: ");
	//distanceDump(ir_3);
	//rprintfCRLF();
	// -------- End   Sharp GP2-------
	
	//-45 degrees
	//distanceRead(ir_4);
	//rprintf("\tir_4: ");
	//distanceDump(ir_4);

	//iter += 0.02;
	//print_map();
	counter++;
	return 0;
}
