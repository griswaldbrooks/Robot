/****************************************************************************

*

*   Copyright (c) 2008 www.societyofrobots.com

*   (please link back if you use this code!)

*

*   This program is free software; you can redistribute it and/or modify

*   it under the terms of the GNU General Public License version 2 as

*   published by the Free Software Foundation.

*

*   Alternatively, this software may be distributed under the terms of BSD

*   license.

*

****************************************************************************/

/******define servo functions******/

//define electronics/motors/servos to specific hardware pins

#define wheel_Left(position)            servo(PORTC,2,position)

#define carbon_Servo(position)			servo(PORTH,2,position)

#define wheel_Right(position)           servo(PORTE,2,position)

#define ir_servo(position)				servo(PORTH,3,position)

/******

ACTUATORS

left wheel    C2

left elbow    C3

left shoulder    C4

right wheel    E2

right elbow    E3

right shoulder     E4

CO2 servo    H2

IR servo    H3

SENSORS

left encoder

    ChB    7

    ChA    8

    Quad    9

right encoder

    ChB    3

    ChA    1

    Quad    0

uv_tron        15

ir_distance    13

COMM

bluetooth    uart0

 

******/

/***********define UART************/

/*int bluetooth=0;

int USB=1;

int uart2=2;

int black_fin=3*/

/**********************************/

/*********define positions*********/

/*long int arm_left_base=600;

long int arm_left_J1=600;

long int arm_left_J2=600;

long int wheel_left=600;

long int wheel_right=600;

long int camera_hor=600;

long int camera_ver=600;*/

/**********************************/
