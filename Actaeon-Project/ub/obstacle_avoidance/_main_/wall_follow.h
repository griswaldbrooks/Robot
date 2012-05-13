/*!	\file	wall_follow.h
 *	\brief	Wall following algorithm set.
 */
 
#ifndef			wall_follow_h
#define			wall_follow_h

#include		<math.h>

#ifdef			ARDUINO_PLATFORM
#include		<wprogram.h>
#endif


#define			nScans		360
#define			rel2head(a)	(a<180)?(a*M_PI/180):((a*M_PI/180)-2*M_PI)
#define			robotwidth 	50
#define			scaling		2E0
#define			blinder		45

//double			_omega;
//double			_veloc;


void navupdate( unsigned int ranges[nScans], double* _veloc, double* _omega ) {
	//.RESET PICK-OFFS
	*_omega = 0; 
	*_veloc = 0;

	double	total_pot        ,
			effective_pot 	 ;

	//.INITIAL RANGE-SUM
	for( int i=0; i<nScans; i++ ) {
		if( ranges[i] > 0 ) {
			total_pot+=ranges[i];
		}
	}

	double left_fan  = 0;
	double right_fan = 0;
	for( int i=0; i<nScans; i++ ) {
		
		
		//.POTENTIAL FIELD AREA
		//effective_pot = (ranges[i]-robotwidth)/total_pot;
		//*_omega += rel2head(i)/effective_pot;
		
		
		#define	front 	30		// cutoff backward fan range
		#define	cut		90		// cutoff frontward fan range


		//.SIMPLE L-R WALL FOLLOWING
		if( (i>front) && (i<180-cut)){
			if( ranges[i] > 0 && ranges[360-i-1] > 0 ) {
				left_fan += (ranges[i]-ranges[360-i-1])/scaling;
			}
		}
		if( (i>180+cut) && (i<front)){
			if( ranges[i] > 0 && ranges[360-i-1] > 0 ) {
				right_fan += (ranges[i]-ranges[360-i-1])/scaling;
			}
		}


		//.VELOCITY RECALCULATION
		if( i<20 ){
			if( ranges[i] > 0 && ranges[360-i-1] > 0 ) {
				*_veloc += (ranges[i]-2*robotwidth)+(ranges[360-i-1]-2*robotwidth);
			} else {
				*_veloc -= 1;
			}
		}
	}
	*_omega = (-left_fan+right_fan)/total_pot;
	*_veloc /= 1000;
}




#endif
