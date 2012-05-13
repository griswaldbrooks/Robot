void control()
{
unsigned right_rotHi = 0;

unsigned right_rotLo = 0;

int rightCount = 0;

unsigned left_rotHi = 0;

unsigned left_rotLo = 0;

int leftCount = 0;

int right_rotState =a2dConvert8bit(0);//right rotations

//int  se1 =a2dConvert8bit(1);

int  right_direction =a2dConvert8bit(3);//right direction, forward = low, reverse = hi

int  left_direction =a2dConvert8bit(7);//left direction, forward = hi, reverse = low

int  left_rotState =a2dConvert8bit(8);//left rotations

//int  se5 =a2dConvert8bit(9);

while( uart1GetByte() != 'x')
{

    if( 0 <= right_rotState && 10 > right_rotState ) //if rotState is low

    {

        right_rotLo++;

    } 

    if( 250 < right_rotState && 255 >= right_rotState ) //if rotState is high

    {

        right_rotHi++;

    }

    if( 0 <= left_rotState &&  10 > left_rotState) //if rotState is low

    {

        left_rotLo++;

    } 

    if( 250 < left_rotState &&  255 >=left_rotState) //if rotState is high

    {

        left_rotHi++;

    }

//---------------------------

    if( right_rotLo > 0 && right_rotHi > 0)//if the encoder has seen both white and black

    {

        if( 0 <= right_direction && right_direction < 10)

        {

            rightCount++;

        }

        if( 250 < right_direction && right_direction <= 255)

        {

            rightCount--;

        }

        right_rotLo = 0;

        right_rotHi = 0;

    }

    if( left_rotLo > 0 && left_rotHi > 0)//if the encoder has seen both white and black

    {

        if( 0 <= left_direction && left_direction  < 10)

        {

            leftCount--;

        }

        if( 250 < left_direction && left_direction  <= 255)

        {

            leftCount++;

        }

        left_rotLo = 0;

        left_rotHi = 0;

    }

   rprintf(" %d, %d, %d, %d \n %d %d %d %d \n",left_rotLo,left_rotHi, left_direction,leftCount,right_rotLo,right_rotHi,right_direction,rightCount);
   rprintf("LEFT DIR:%d, LEFT DST:%d, RIGHT DIR:%d, RIGHT DST:%d\n,",left_direction,leftCount,right_direction, rightCount);

    

    right_rotState =a2dConvert8bit(0);//right rotations

    right_direction =a2dConvert8bit(3);//right direction, forward = low, reverse = hi

    left_direction =a2dConvert8bit(7);//left direction, forward = hi, reverse = low

    left_rotState =a2dConvert8bit(8);//left rotations

} 
/*
unsigned right_rotHi = 0; 	

        if( 250 < left_direction && left_direction  <= 255) {leftCount++;}

        left_rotLo = 0;

        left_rotHi = 0;

    }
	//rprintf(" %d, %d, %d, %d \n %d %d %d %d \n",left_rotLo,left_rotHi, left_direction,leftCount,right_roLo,right_rotHi,right_direction,rightCount);
    //rprintf("LEFT DIR:%d, LEFT DST:%d, RIGHT DIR:%d, RIGHT DST:%d\n,",left_direction,leftCount,right_direction, rightCount);

	right_rotState =a2dConvert8bit(0);//right rotations
    right_direction =a2dConvert8bit(3);//right direction, forward = low, reverse = hi
    left_direction =a2dConvert8bit(7);//left direction, forward = hi, reverse = low
    left_rotState =a2dConvert8bit(8);//left rotations
}

int control_PID( )
{	

}

 void update_motors()
{
//Right 607-640 braking range
//left    613-633 braking range
        if(motorLeftSpeed<613||motorLeftSpeed>633)
        {
            wheel_Left(motorLeftSpeed);
        }
        if(motorRightSpeed<607||motorRightSpeed>640)
        {
            wheel_Right(motorRightSpeed);
        }  
}*/
}
