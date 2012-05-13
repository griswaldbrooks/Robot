/*
LAST UPDATE: 06/08/10

- change the turnAround function so it turns intelligently to somewhere that is empty
-
*/


#define MAP_X                   30
#define MAP_Y                   30

#define WALL                    254
#define GOAL                    253
#define NOTH					0

#define ROBOT_DISTX             1
#define ROBOT_DISTY             1
#define NORTH                   0
#define EAST                    1
#define SOUTH                   2
#define WEST                    3


// Place any #define statements here before you include ANY other files
// You must ALWAYS specify the board you are using
// These are all in the 'sys' folder e.g.

#include "hardware.h"
#include "math.h"

// Initialise the hardware
// Now create any global variables such as motors, servos, sensors etc
// This routine is called once only and allows you to do set up the hardware
// Dont use any 'clock' functions here - use 'delay' functions instead
void appInitHardware(void) {
	initHardware();
}

// Initialise the software
TICK_COUNT appInitSoftware(TICK_COUNT loopStart){
	return 0;
}

// This routine is called once to allow you to set up any other variables in your program
// You can use 'clock' function here.
// The loopStart parameter has the current clock value in uS

	
//robot start values
int courseMap[MAP_X][MAP_Y] = {};
char robotX = 5;
char robotY = 15;

int ir_degree = 90;
int scan_width = 45;
int scan_center = 90;
char scan_dir = 0; //0 = scan right, 1 = scan left

int turn_speed = 10;

int speed = 0;
int turn_timeMS = 0;
char turnLeft = 0;
char turnRight = 0;
int objectAccum = 0;


//***********TRIG LOOKUP TABLES**********
//returns a trig sin or cos calculation value multiplied by 100 (to avoid floating point math)
//returns a trig tan calculation value multiplied by 10 (to avoid floating point math)
//only allows for angles between 0 and 360 degrees

//multiplied by 100 so no floating point math
signed int angtable[73]={100,100,98,97,94,91,87,82,77,71,64,57,50,42,34,26,17,9,0,-9,-17,-26,-34,-42,-50,-57,-64,-71,-77,-82,-87,-91,-94,-97,-98,-100,
						 -100,-100,-98,-97,-94,-91,-87,-82,-77,-71,-64,-57,-50,-42,-34,-26,-17,-9,0,9,17,26,34,42,50,57,64,71,77,82,87,91,94,97,98,100,100};

signed int cos_SoR(long signed int degrees)//returns cos*100
	{
	if (degrees >= 0)//positive angles
		return angtable[degrees/5];
	else
		return -angtable[72-(-degrees)/5];
	}

signed int sin_SoR(long signed int degrees)//returns sin*100
	{
	degrees=degrees - 90;//phase shift 90 degrees

	if (degrees >= 0)//positive angles
		return angtable[degrees/5];
	else
		return -angtable[72-(-degrees)/5];
	}

signed int tan_SoR(long signed int degrees)//returns tan * 10
	{
	//tan(x) = sin(x)/cos(x)
	if (degrees == 90 || degrees == -90 || degrees == 270 || degrees == -270)//blows up
		return 0;//what else should I return?!?!?
	return sin_SoR(degrees)/cos_SoR(degrees)*10;
	}

//***************************************



signed long int degreeToSpeed(signed long int degree)
{
	return ((degree * (DRIVE_SPEED_MAX - DRIVE_SPEED_MIN))/180) + DRIVE_SPEED_MIN;
}

void update_map_ir(short startAngle, short endAngle)
{
    int wallRow;
    int wallCol;
	short zone0 = 0;
	 
    act_setSpeed(&ir_servo, degreeToSpeed(startAngle));
	delay_ms(20);

    for(int currentAngle = startAngle; currentAngle <= endAngle; currentAngle += 5)
    { 
        distanceRead(ir_dist);
		act_setSpeed(&ir_servo, degreeToSpeed(currentAngle));

		if((ir_dist.distance.cm >= 10) && (ir_dist.distance.cm <= 80))
		{
        	

        	wallCol = ((ir_dist.distance.cm/5.0) * (cos_SoR(currentAngle)))/100;
        	wallRow = ((ir_dist.distance.cm/5.0) * (ABS(sin_SoR(currentAngle))))/100;
        	courseMap[robotX + wallRow][robotY + wallCol] = WALL;
        	rprintf("wallCol: %d\twallRow: %d\tangle: %d",wallCol,wallRow,currentAngle); distanceDump(ir_dist);rprintf("\n");
		}
		zone0 = isAnythingZone0();
		if(zone0 !=0)
		{
			act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
			act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		}
		delay_ms(20);
    }

	act_setSpeed(&ir_servo, degreeToSpeed(endAngle));
	delay_ms(20);

	for(int currentAngle = endAngle; currentAngle >= startAngle; currentAngle -= 5)
    { 
        distanceRead(ir_dist);
		act_setSpeed(&ir_servo, degreeToSpeed(currentAngle));

		if((ir_dist.distance.cm >= 10) && (ir_dist.distance.cm <= 80))
		{
        	

        	wallCol = ((ir_dist.distance.cm/5.0) * (cos_SoR(currentAngle)))/100;
        	wallRow = ((ir_dist.distance.cm/5.0) * (ABS(sin_SoR(currentAngle))))/100;
        	courseMap[robotX + wallRow][robotY + wallCol] = WALL;
        	rprintf("wallCol: %d\twallRow: %d\tangle: %d",wallCol,wallRow,currentAngle); distanceDump(ir_dist);rprintf("\n");
		}

		delay_ms(20);
    }
}

void printMap(int robotOrient)
//clears the map of propagated values
{
    //delay_ms(100);
    //system("cls");
    for (int row=0; row < MAP_X; row++)
    //traverse MAP_X rows
    {
        for (int col=0; col < MAP_Y; col++)
        //traverse MAP_Y columns
        {
            if(row== robotX && col == robotY)
            //if the node is the robot print direction instead of the number
                {
                    if(robotOrient == NORTH)
                        {rprintf(" ^ ");}
                    else if(robotOrient == EAST)
                        {rprintf(" > ");}
                    else if(robotOrient == SOUTH)
                        {rprintf(" V ");}
                    else
                        {rprintf(" < ");}
                }
//-----------print robot block
            else if((row == (robotX - ROBOT_DISTX)) && col == (robotY - ROBOT_DISTY))
                {rprintf(" R ");}
            else if((row == (robotX - ROBOT_DISTX)) && col == (robotY))
                {rprintf(" R ");}
            else if(row == (robotX - ROBOT_DISTX) && col == (robotY + ROBOT_DISTY))
                {rprintf(" R ");}
            else if(row == robotX && col == (robotY - ROBOT_DISTY))
                {rprintf(" R ");}
            else if(row == robotX && col == (robotY + ROBOT_DISTY))
                {rprintf(" R ");}
            else if(row == (robotX + ROBOT_DISTX) && col == (robotY - ROBOT_DISTY))
                {rprintf(" R ");}
            else if(row == (robotX + ROBOT_DISTX) && col == robotY)
                {rprintf(" R ");}
            else if(row == (robotX + ROBOT_DISTX) && col == (robotY + ROBOT_DISTY))
                {rprintf(" R ");}
//-----------print robot block
            else if(courseMap[row][col]==WALL)
            //if the node is a wall print W instead of the number
                {rprintf("WWW");}
            else if(courseMap[row][col]==GOAL)
            //if the node is the goal print G instead of the number
                {rprintf("GGG");}
            else
            //otherwise print the number
                {rprintf(" %d ",courseMap[row][col]);}
        }
        rprintf("\n");//go to the next line since the first line is over
    }
    rprintf("\n");//print an empty line since the map is complete
}

void clearMap()
//clears the map of propagated values
{
 
  for (int row=0; row < MAP_X; row++)
  //traverse MAP_X rows
  {

    for (int col=0; col < MAP_Y; col++)
    //traverse MAP_Y columns
    {
      
	  courseMap[row][col] = NOTH;
	  /*
      if (map[row][col] != WALL)//if the map node contains a WALL, PADD, or GOAL then don't clear it
      {
		map[row][col] = NOTH;
      }
	  */
	  
	}
  }
}


void scanAndDrive()
{
	for(int degree = 0; degree < 180; degree+= 10)
	{
		distanceRead(ir_dist);
		act_setSpeed(&ir_servo, degreeToSpeed(degree));

		if((ir_dist.distance.cm >= 10) && (ir_dist.distance.cm <= 80))
		{
			distanceDump(ir_dist);
			rprintf("DEGREE:%d COS:%d SIN:%d\n", degree, cos_SoR(degree), ABS(sin_SoR(degree)));
		}
		
		
		delay_ms(100);
		
	}

}

void driveVel()
{

	rprintf("\n\n\n\n\n");
/*	for(int x = -127; x < 127; x++)

	{

    	act_setSpeed(&wheel_left,x);
		act_setSpeed(&wheel_right,0);

     	rprintfu32(clockGetus()); rprintf(",");

    	rprintf("%d,",x);

    	encoderRead(quad_left);
    	encoderDump(quad_left);rprintf(",");

    	delay_ms(1000);

    	rprintfu32(clockGetus()); rprintf(",");

    	encoderRead(quad_left);
    	encoderDump(quad_left);rprintf("\n");

	}
*/
	for(int y = -127; y < 127; y++)

	{

    	act_setSpeed(&wheel_right,y);
		act_setSpeed(&wheel_left,0);

      	rprintfu32(clockGetus()); rprintf(",");

    	rprintf("%d,",y);

    	encoderRead(quad_right);
    	encoderDump(quad_right);rprintf(",");

    	delay_ms(1000);

    	rprintfu32(clockGetus()); rprintf(",");

	
    	encoderRead(quad_right);
    	encoderDump(quad_right);rprintf("\n");

	}    

    

    

    /*

    encoderRead(quad_left);

    encoderRead(quad_right);

    ENCODER_TYPE l_ticks = quad_left.encoder.value;

    ENCODER_TYPE r_ticks = quad_right.encoder.value;

    int left_ticks = 8;

    int right_ticks = 8;

    

    encoderDump(quad_right);

    encoderDump(quad_left);

    */


}

int isAnythingZone0()
{//zone 0 is a 3 row by 5 col block directly if front
	int zone0 = 0;
	
	for(short zon0row = 0; zon0row < 5; zon0row++)
	{
		for(short zon0col = 0; zon0col < 3; zon0col++)
		{
			zone0 += courseMap[robotX + (zon0row + ROBOT_DISTX + 1)][robotY + (zon0col - 1)];
		} 
	}

	return zone0;
}

int isAnythingZone1()
{//zone 1 is four rows in front
	int zone1 = 0;
	
	for(short zon1row = 0; zon1row < 5; zon1row++)
	{
		zone1 += courseMap[robotX + (ROBOT_DISTX + 1 + 5 + zon1row)][robotY];
	}

	return zone1;
}

int isAnythingZone2()
{//zone 2 is four rows in front and 3 cols right of center
	int zone2 = 0;
	
	for(short zon2row = 0; zon2row < 5; zon2row++)
	{
		zone2 += courseMap[robotX + (ROBOT_DISTX + 1 + 5 + zon2row)][robotY - 1];
	}

	return zone2;
}

int isAnythingZone3()
{ //zone 3 is four rows in front and 3 cols left of center
	int zone3 = 0;
	
	for(short zon3row = 0; zon3row < 5; zon3row++)
	{
		zone3 += courseMap[robotX + (ROBOT_DISTX + 1 + 5 + zon3row)][robotY + 1];
	}

	return zone3;
}

int isAnythingZone4()
{ //zone 3 is four rows in front and 3 cols left of center
	int zone4 = 0;
	
	for(short zon4row = 0; zon4row < 10; zon4row++)
	{
		zone4 += courseMap[robotX + (ROBOT_DISTX + 1 + 0 + zon4row)][robotY - 2];
	}

	return zone4;
}

int isAnythingZone6()
{ //zone 3 is four rows in front and 3 cols left of center
	int zone6 = 0;
	
	for(short zon6row = 0; zon6row < 5; zon6row++)
	{
		zone6 += courseMap[robotX + (ROBOT_DISTX + 1 + 0 + zon6row)][robotY - 3];
	}

	return zone6;
}

int isAnythingZone5()
{ //zone 3 is four rows in front and 3 cols left of center
	int zone5 = 0;
	
	for(short zon5row = 0; zon5row < 5; zon5row++)
	{
		zone5 += courseMap[robotX + (ROBOT_DISTX + 1 + 0 + zon5row)][robotY + 2];
	}

	return zone5;
}

int isAnythingZone7()
{ //zone 3 is four rows in front and 3 cols left of center
	int zone7 = 0;
	
	for(short zon7row = 0; zon7row < 5; zon7row++)
	{
		zone7 += courseMap[robotX + (ROBOT_DISTX + 1 + 0 + zon7row)][robotY + 3];
	}

	return zone7;
}

int isAnythingLeftZone()
{ //zone 3 is four rows in front and 3 cols left of center
	int zoneLeft = 0;
	
	for(short zonLeftRow = 0; zonLeftRow < 3; zonLeftRow++)
	{
		for(short zonLeftCol = 0; zonLeftCol < 5; zonLeftCol++)
		{
			zoneLeft += courseMap[robotX + (zonLeftRow)][robotY + (ROBOT_DISTY + 1 + zonLeftCol)];
		}
	}

	return zoneLeft;
}

int isAnythingRightZone()
{ //zone 3 is four rows in front and 3 cols left of center
	int zoneRight = 0;
	
	for(short zonRightRow = 0; zonRightRow < 3; zonRightRow++)
	{
		for(short zonRightCol = 0; zonRightCol < 5; zonRightCol++)
		{
			zoneRight += courseMap[robotX + (zonRightRow)][robotY - (ROBOT_DISTY + 1 + zonRightCol)];
		}
	}

	return zoneRight;
}

void turn_around()
{
	turn_aroundTime();
	/* until encoder is fixed
	short wheel_ticks = 64;

	encoderRead(quad_right);
	encoderRead(quad_left);
	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);

	ENCODER_TYPE rightStart = quad_right.encoder.value;
	ENCODER_TYPE leftStart = quad_left.encoder.value;

	
	while(quad_left.encoder.value < (leftStart + wheel_ticks) ||
		  quad_right.encoder.value > (rightStart - wheel_ticks))
	{
		act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/24);
		act_setSpeed(&wheel_right, DRIVE_SPEED_MIN/24);
		encoderRead(quad_right);
		encoderDump(quad_right);
		encoderRead(quad_left);
		encoderDump(quad_left);

		if(quad_left.encoder.value < (leftStart + wheel_ticks))
		{
			rprintf("LEFT BRAKE\n");
			act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		}
		if(quad_right.encoder.value > (rightStart - wheel_ticks))
		{
			rprintf("RIGHT BRAKE\n");
			act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		}
		
	}

	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
	*/
}


void turn_aroundTime()
{
	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
	
	act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/24);
	act_setSpeed(&wheel_right, DRIVE_SPEED_MIN/24);

	delay_ms(2000);

	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);

}

void turn_rightTime(int turnSpeed, int timeMS)
{
	act_setSpeed(&wheel_left, turnSpeed); //the closer the object, the faster the turning
	//act_setSpeed(&wheel_right, DRIVE_SPEED_MAX - turnSpeed);

	delay_ms(timeMS);
}


void turn_right()
{
	turn_rightTime(5, 2000);
	
	/* until encoder is replaced
	
	short wheel_ticks = 32;

	encoderRead(quad_right);
	encoderRead(quad_left);
	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);

	ENCODER_TYPE rightStart = quad_right.encoder.value;
	ENCODER_TYPE leftStart = quad_left.encoder.value;

	
	while(quad_left.encoder.value < (leftStart + wheel_ticks) ||
		  quad_right.encoder.value > (rightStart - wheel_ticks))
	{
		act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/24);
		act_setSpeed(&wheel_right, DRIVE_SPEED_MIN/24);
		encoderRead(quad_right);
		encoderDump(quad_right);
		encoderRead(quad_left);
		encoderDump(quad_left);

		if(quad_left.encoder.value < (leftStart + wheel_ticks))
		{
			rprintf("LEFT BRAKE\n");
			act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		}
		if(quad_right.encoder.value > (rightStart - wheel_ticks))
		{
			rprintf("RIGHT BRAKE\n");
			act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		}
		
	}

	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
	*/
}

void turn_leftTime(int turnSpeed, int timeMS)
{
	//act_setSpeed(&wheel_left, DRIVE_SPEED_MAX - turnSpeed); //the closer the object, the faster the turning
	act_setSpeed(&wheel_right, turnSpeed);

	delay_ms(timeMS);
}

void turn_left()
{
	turn_leftTime(5, 2000);
	
	/* until encoder is replaced
	short wheel_ticks = 32;

	encoderRead(quad_right);
	encoderRead(quad_left);
	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);

	ENCODER_TYPE rightStart = quad_right.encoder.value;
	ENCODER_TYPE leftStart = quad_left.encoder.value;

	
	while(quad_left.encoder.value > (leftStart - wheel_ticks) ||
		  quad_right.encoder.value < (rightStart + wheel_ticks))
	{
		act_setSpeed(&wheel_left, DRIVE_SPEED_MIN/24);
		act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/24);
		encoderRead(quad_right);
		encoderDump(quad_right);
		encoderRead(quad_left);
		encoderDump(quad_left);

		if(quad_left.encoder.value > (leftStart - wheel_ticks))
		{
			rprintf("LEFT BRAKE\n");
			act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		}
		if(quad_right.encoder.value < (rightStart + wheel_ticks))
		{
			rprintf("RIGHT BRAKE\n");
			act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		}
		
	}

	act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
	act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
	*/
}


void turnToNothingAndGo()
{
	
	while(true)
	{
		act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		clearMap();
		update_map_ir(0, 180); //scan 180
		//printMap(SOUTH);
		//is left zone clear? 
		int leftZone = isAnythingLeftZone();
		if(leftZone == 0)
		{
			turn_left();
			return;
		}
		//is right zone clear?
		int rightZone = isAnythingRightZone();
		if(rightZone == 0)
		{
			turn_right();
			return;
		}
		
		//turn and try again
		if(rightZone < leftZone)
		{
			turn_right();
		
		}
		else
		{
			turn_left();
		}	

	}

}

void driveStraight(int speed)
{
	
	act_setSpeed(&wheel_left, speed);
	act_setSpeed(&wheel_right, speed);

}

void driveForward()
{

	
	
	//if anything is in zone 0, stop and rescan 
	//if anything is still in zone one, turn around

	int zone0 = isAnythingZone0();

	
	if(zone0 != 0)
	{
		//stop
		act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);
		
		//rescan
		clearMap();
		update_map_ir(45, 135); //scan 45 degrees either side of center line
		printMap(SOUTH);
		zone0 = isAnythingZone0();
		
		if(zone0 != 0)
		{
			//turn_around();
			turnToNothingAndGo();
			return;
		}

	}

	else //if nothing is in zone 0 
	{	
	
		short zone1 = isAnythingZone1();
		short zone2 = isAnythingZone2();
		short zone3 = isAnythingZone3();
		short zone4 = isAnythingZone4();
		short zone5 = isAnythingZone5();
		short zone6 = isAnythingZone6();
		short zone7 = isAnythingZone7();
		
		if(zone1 != 0)
		{//left 4
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/128);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/1);
			delay_ms(500);
		}
		else if(zone2 != 0)
		{//left 3
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/64);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/2);
			delay_ms(500);
		}
		else if(zone4 != 0)
		{//left 2
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/32);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/4);
			delay_ms(500);
		}
		else if(zone6 != 0)
		{//left1
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/16);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/8);
			delay_ms(500);
		}
		else if(zone3 != 0)
		{//right 3
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/2);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/64);
			delay_ms(500);
		}
		else if(zone5 != 0)
		{//right 2
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/4);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/32);
			delay_ms(500);
		}
		else if(zone7 != 0)
		{//right 1
			act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/8);
			act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/16);
			delay_ms(500);
		}
		
		act_setSpeed(&wheel_left, DRIVE_SPEED_MAX/10);
		act_setSpeed(&wheel_right, DRIVE_SPEED_MAX/10);
		return;		
			
	}

}

void armTester()
{
	for(int speed = DRIVE_SPEED_MIN; speed < DRIVE_SPEED_MAX; speed++)
	{
		act_setSpeed(&left_elbow, speed);
		act_setSpeed(&left_shoulder, speed);
		act_setSpeed(&right_elbow, speed);
		act_setSpeed(&right_shoulder, speed);
		delay_ms(50);
	}
}

/*robot start values
int ir_degree = 0;
int scan_width = 45;
int scan_center = 90;
char scan_dir = 0; //0 = scan right, 1 = scan left
int speed_divisor  = 10;
int turn_timeMS = 0;
char turnLeft = 0;
char turnRight = 0;
int objectAccum = 0;
*/


void scan_once()
{
	
	act_setSpeed(&ir_servo, degreeToSpeed(ir_degree));
	rprintf("%d\n",ir_degree);
	delay_ms(20);
	distanceRead(ir_dist);

	if((ir_dist.distance.cm >= 10) && (ir_dist.distance.cm <= 80)) //if the reading is within viewable range
	{
		if(ir_degree > scan_center) //then the object is on the right side
		{
			turnLeft = 1;
			
		}
		else if (ir_degree < scan_center)//it is on the left side
		{
			turnRight = 1;
		}
		
		turn_speed = (DRIVE_SPEED_MAX / (9 - ((90 - ir_dist.distance.cm)/10))); //the closer the object, the smaller the divisor
		
		speed = (DRIVE_SPEED_MAX / (9 - (ir_dist.distance.cm/10)));
		turn_timeMS = 1000/(abs(scan_center - ir_degree) + 1); //the more in front of the robot the object is, the longer the turn
        
	}
	

	//update next positon
	if(ir_degree > scan_center + scan_width)
	{
		scan_dir = 1;
	
	}
	else if(ir_degree < scan_center - scan_width)
	{
		scan_dir = 0;
		
	}

	if(scan_dir == 0)
	{
		ir_degree = ir_degree + 10;
	}
	else
	{
		ir_degree = ir_degree - 10;
	}

	
}


TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart){

	
	
	rprintfProgStrM("Program start\n");
	
	act_setSpeed(&left_elbow, degreeToSpeed(120));
	act_setSpeed(&left_shoulder, degreeToSpeed(90));
	act_setSpeed(&right_elbow, degreeToSpeed(80));
	act_setSpeed(&right_shoulder, degreeToSpeed(90));
	
	rprintf("scan once start\n");
	scan_once();
	rprintf("scan once stop\n");
	rprintf("turn speed: %d speed: %d\n",turn_speed, speed);
	rprintf("time: %d\n", turn_timeMS);
	/*
	if((ir_dist.distance.cm >= 10) && (ir_dist.distance.cm <= 15))
	{
		act_setSpeed(&wheel_left, DRIVE_SPEED_BRAKE);
		act_setSpeed(&wheel_right, DRIVE_SPEED_BRAKE);

		rprintf("update map\n");
		update_map_ir(45, 135); //scan 45 degrees either side of center line
		//printMap(SOUTH);
		
		rprintf("drive forward\n");
		driveForward();
		rprintf("clear map\n");
		clearMap();
		
	}
	*/
	if(turnLeft == 1)
	{
		rprintf("turn left start\n");
		turn_leftTime(turn_speed,turn_timeMS);
		rprintf("turn left stop\n");
	}
	if(turnRight == 1)
	{
		rprintf("turn right start\n");
		turn_rightTime(turn_speed,turn_timeMS);
		rprintf("turn right stop\n");
	}

	turnLeft = 0; //clear turn flags
	turnRight = 0;

	rprintf("drive straight start\n");
	driveStraight(speed);
	rprintf("drive straight stop\n");


/*
	while(true)
	{
		rprintf("update map\n");
		update_map_ir(45, 135); //scan 45 degrees either side of center line
		//printMap(SOUTH);
		
		rprintf("drive forward\n");
		driveForward();
		rprintf("clear map\n");
		clearMap();
	}
	//scanAndDrive();
	//driveVel();
	
	rprintf("Program terminated.\n");
	delay_ms(1000);
*/	
	rprintf("App loop end\n");
	return 0;

}
