/*!	\file
 *	\brief 
 */
 
#ifndef  	obstacle_avoidance_defines_h
#define  	obstacle_avoidance_defines_h

#include 	<math.h>

/*-CONSTANT-----------------------------------------------------------------------------*/
/*!	\def 	frames_NEXPECTED
 *	\brief	Number of expected frames expected in one range-finder sweep
 *			A frame is considered to be an input of the format {radius,theta}
 *	\see 	addFrame( double radius, double theta )
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	veloc_CUTOFF
 *	\brief	Maximum achievable velocity solution
 *			Solution values above this cutoff value will be forced
 *			to a value at or below the cutoff level.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	veloc_LINEARCOEFF
 *	\brief	Linear coefficient applied to velocity output solution
 *			Can be utilized to make proportional adjustments to output magnitudes.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	veloc_ANGLESCOPE
 *	\brief	Angular region considered during velocity calculation
 *			Represents sweep region from { -(SCP)<THETA<+(SCP) }
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	omega_CUTOFF
 *	\brief	Maximum achievable omega solution
 *			Solution values above this cutoff value will be forced
 *			to a value at or below the cutoff level.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	omega_LINEARCOEFF
 *	\brief	Linear coefficient applied to omega output solution
 *			Can be utilized to make proportional adjustments to output magnitudes.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	omega_RADSCOPE
 *	\brief	Radial scoping boundary condition
 *			Utilized to provide ideal standard for potential function as well
 *			as to select points for aperature detection
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	fndef_FIELDBIAS
 *	\brief	Positive-negative bias factor for potential field calculation
 *	\see	_POTENTIAL_FIELD(tht,rad)
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	fndef_LOGITCUTOFF
 *	\brief	Cutoff value for _LOGIT(n) based classifcation
 *	\see	_LOGIT(n)
 */
/*-FUNCTIONAL---------------------------------------------------------------------------*/
/*!	\def 	_LOGIT(n)
 *	\brief	Log-odds function
 *	\param	[in]	n	value between {0<n<1}
 *	\return	value between {-inf<output<inf}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_rfang(tht)
 *	\brief	Potential function component from sweep angle, theta
 *			#For angles ||tht|| > pi/2, potentials can be negative
 *	\param	[in]	tht		theta of angular sweep {-pi<tht<pi}
 *	\return	value between {-1<output<1}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_rfrad(rad)
 *	\brief	Potential function component from range finder radius value
 *	\param	[in]	rad		value between {0<n<inf}
 *	\return	value between {0<output<1}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_FIELD(tht,rad)
 *	\brief	Full potential field function
 *			Positive-Negative bias factor can be adjusted by chaing the value of
 *			fndef_FIELDBIAS:
 *			{+} when fndef_FIELDBIAS > 0.5, potentials will be predominantly positive
 *			{+} when fndef_FIELDBIAS < 0.5, potentials will be predominantly negative
 *			{+} positive value correspond to directional acceptance
 *			{+} negative value correspond to directional rejection
 *			In turn, robot will move away from objects directly behind it and head
 *			towards the most open part of its current field of view.
 *	\param	[in]	rad		value between {0<n<inf}
 *	\param	[in]	tht		theta of angular sweep {-pi<tht<pi}
 *	\return	value between {-2<output<2}
 *  \see	_POTENTIAL_rfrad(rad)
 *	\see	_POTENTIAL_rfang(tht)
 *	\see	fndef_FIELDBIAS
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_GETOMEGA
 *	\brief	Gets most recent output for omega
 *	\return	omega
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_GETVELOC
 *	\brief	Potential function component from range finder radius value
 *	\param	[in]	rad		value between {0<n<inf}
 *	\return	value between {0<output<1}
 */
/*--------------------------------------------------------------------------------------*/

#define		frames_NEXPECTED			36

#define	 	veloc_CUTOFF				1
#define	 	veloc_LINEARCOEFF			1
#define	 	veloc_ANGLESCOPE			1

#define	 	omega_CUTOFF				1
#define	 	omega_LINEARCOEFF			1
#define	 	omega_RADSCOPE				1

#define	 	fndef_FIELDBIAS				0.5
#define	 	fndef_LOGITCUTOFF			1E2

#define		_LOGIT(n)					log(n)-log(1-n)
#define		_POTENTIAL_rfang(tht)		cos(tht)					
#define		_POTENTIAL_rfrad(rad)		1-exp(rad/omega_RADSCOPE)
#define		_POTENTIAL_FIELD(tht,rad)	_POTENTIAL_rfang(tht) + _POTENTIAL_rfrad(rad) + fndef_FIELDBIAS

#define		_GETOMEGA
#define		_GETVELOC

#endifs