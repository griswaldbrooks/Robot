/*
 * sentinel.c
 *
 * Created: 9/23/2011 12:41:56 AM
 *  Author: Brian Cairl
 */ 
#include<SoR_Utils.h>
#include <avr/io.h>
#include <math.h>

/* Containers:
===================================================================================================================================================*/

// 3x1 Vector
typedef struct {
	double 	e11, e21, e31;
} vec3x1;

// 3x3 Matrix
typedef struct {
	double 	e11, e12, e13, e21, e22, e23, e31, e32, e33;
} mat3x3;

/*=================================================================================================================================================*/

// Math constants
#define pi			3.146
#define r2d_const	180/pi
#define d2r_const	pi/180

// Array/Vector Definitions
#define empty1x4 	{ 0, 0, 0, 0 }
#define empty3x1 	{ 0, 0, 0 }
#define ones3x1 	{ 1, 1, 1 }



// Matrix Definitions
#define empty3x3	{ 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define Rx( A )		{ 1, 0, 0, 0, cos(A), -sin(A), 0, sin(A), cos(A) } 
#define Ry( A )		{ cos(A), 0, sin(A), 0, 1, 0, -sin(A), 0, cos(A) } 
#define Rz( A )		{ cos(A), -sin(A), 0, sin(A), cos(A), 0, 0, 0, 1 } 


/*=================================================================================================================================================*/
/* 
Source Name		:	geometry.c
Description		:	general shortcuts, trigonometry
Functions  		:	degrees, radians, sign, cosd, sind, tand, acosd, asind, atand, atand2, acos3, side law
Author			:	Brian Cairl
Start Date		:	9/21/2011
*/
/*=================================================================================================================================================*/






/*  
	Function	: 'acos3' 
	Full		: 'arccosine 3'
	Description	:  Returns an angle within a triangle between sides 'a' and 'b', opposite a side 'c.'
	Rules		:  Parameters 'a' and 'b' must not exceed the magnitude of 'c' and vice versa.
	Input Type	:  (double), (double), (double)
	Return Type :  (double)
*/

double acos3( double a, double b, double c )
{	
	double temp = ( pow( c, 2 ) - pow( a, 2 ) - pow( b, 2 ) )/(-2*a*b);
	return( atan2( sqrt( 1 - pow( temp, 2 ) ), temp ) );
}






/*  
	Function	: 'sidelaw' 
	Full		: 'side obtainment from law of cosines'
	Description	:  Returns length of side opposite angle 'A'
	Rules		:  0 < a < 180
	Input Type	:  (double), (double), (double)
	Return Type :  (double)
*/

double sidelaw( double x, double y, double A )
{	
	return( sqrt( pow( x, 2 )+pow( y, 2 ) + 2*x*y*cos( A ) ) );
}




/*
Shortcuts:		name		input							operation									description
-------------------------------------------------------------------------------------------------------------------------------------------------- */
double 			degrees		( double _radians )				{ return( _radians*r2d_const ); }			// converts radians to degrees
double 			radians		( double _degrees ) 			{ return( _degrees*d2r_const ); }			// converts degrees to radians
int 			sign		( double input   ) 				{ return( trunc((input/fabs(input))) ); }	// extracts input sign {-1, 1 }

double 			cosd		( double input   ) 				{ return( cos( radians( input ) ) );  }		// degree-input based cosine
double 			sind		( double input   ) 				{ return( sin( radians( input ) ) );  }		// degree-input based sine 
double 			tand		( double input   ) 				{ return( tan( radians( input ) ) );  }		// degree-input based tangent

double 			acosd		( double input   ) 				{ return( degrees( acos( input ) ) ); }		// degree-input based arc cosine
double 			asind		( double input   ) 				{ return( degrees( asin( input ) ) ); }		// degree-input based arc sine 
double 			atand		( double input   ) 				{ return( degrees( atan( input ) ) ); }		// degree-input based arc tangent
double 			atand2		( double y, double x  ) 		{ return( degrees( atan2( y, x ) ) ); }		// degree-input based arc tangent
/*------------------------------------------------------------------------------------------------------------------------------------------------ */






/*=================================================================================================================================================*/
/* 
Source Name		:	oparray.c
Description		:	array-based operations
Functions  		:	clone, clonei, ewadd, ewsub, ewmul, ewdiv, ewpow, fill, zero, one
Author			:	Brian Cairl
Start Date		:	9/21/2011
*/
/*=================================================================================================================================================*/








/*  
	Function	: 'clone' 
	Full		: 'shallow copy-based array cloning'
	Description	:  Fills 'array2' with all elements of 'array1'
	Rules		:  'dim' must be consistent
	Input Type	:  (double array), (double array), (int)
	Return Type :  none
*/

void clone( double array1[], double array2[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array2[i] = array1[i];
	}
}




/*  
	Function	: 'clonei' 
	Full		: 'indice-specific shallow copy-based array cloning'
	Description	:  Fills 'array2' with all elements at indices 'i1' to 'i2' of 'array1'
	Rules		:  index parameters 'i1' and 'i2' specify the first and last element considered
	Input Type	:  (double array), (double array), (int)
	Return Type :  none
*/

void clonei( double array1[], double array2[], int i1, int i2 )
{
	for( int i = i1; i <= i2; i++ ){
		array2[i] = array1[i];
	}
}





/*  
	Function	: 'ewadd' 
	Full		: 'element wise addition'
	Description	:  adds elements of 'array1' to corresponding elements of 'array2', filling 'array3'
	Rules		:  'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewadd( double array1[], double array2[], double array3[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array3[i] = array1[i]+array2[i];
	}
}





/*  
	Function	: 'ewsub' 
	Full		: 'element wise subtraction'
	Description	:  subtracts elements of 'array2' to corresponding elements of 'array1', filling 'array3'
	Rules		:  'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewsub( double array1[], double array2[], double array3[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array3[i] = array1[i]-array2[i];
	}
}





/*  
	Function	: 'ewmul' 
	Full		: 'element wise multiplication'
	Description	:  multiplies elements of 'array1' with corresponding elements of 'array2', filling 'array3'
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewmul( double array1[], double array2[], double array3[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array3[i] = array1[i]*array1[2];
	}
}





/*  
	Function	: 'ewdiv' 
	Full		: 'element wise division'
	Description	:  divides elements of 'array1' by corresponding elements of 'array2', filling 'array3'
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewdiv( double array1[], double array2[], double array3[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array3[i] = array1[i]/array2[i];
	}
}





/*  
	Function	: 'ewpow' 
	Full		: 'element wise exponentiation'
	Description	:  raises elements of 'array1' to powers corresponding to elements of 'array2', filling 'array3'
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewpow( double array1[], double array2[], double array3[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array3[i] = pow( array1[i], array2[i] );
	}
}





/*  
	Function	: 'ewabs' 
	Full		: 'element wise exponentiation'
	Description	:  fills 'array2' with absolute values of'array1'
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (double array), (double array), (int)
	Return Type :  none
*/

void ewabs( double array1[], double array2[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array2[i] = fabs( array1[i] );
	}
}





/*  
	Function	: 'fill' 
	Full		: 'repeating value array'
	Description	:  fills 'array1' with a number 'n'
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (double), (int)
	Return Type :  none
*/

void fill( double array1[], double n, int dim )
{
	for( int i = 0; i < dim; i++ ){
		array1[i] = n;
	}
}





/*  
	Function	: 'zero' 
	Full		: 'repeating 0-value array'
	Description	:  fills 'array1' with zeros
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (int)
	Return Type :  none
*/

void zero( double array1[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array1[i] = 0;
	}
}





/*  
	Function	: 'one' 
	Full		: 'repeating 1-value array'
	Description	:  fills 'array1' with ones
	Rules		: 'dim' must be consistent
	Input Type	:  (double array), (int)
	Return Type :  none
*/

void one( double array1[], int dim )
{
	for( int i = 0; i < dim; i++ ){
		array1[i] = 1;
	}
}





/*=================================================================================================================================================*/
/* 
Source Name		:	array2scalar.c
Description		:	vector math, inner element operations
Functions  		:	mag, iesum, iepro
Author			:	Brian Cairl
Start Date		:	9/21/2011
*/
/*=================================================================================================================================================*/





/*  
	Function	: 'mag' 
	Full		: 'vector magnitude'
	Description	:  Returns the magnitude of the elements withing an array, '(double)input_arr[],' treating
				   said elements as vector components
	Rules		:  index parameters 'i1' and 'i2' specify the first and last element considered
	Input Type	:  (double array), (int), (int)
	Return Type :  (double)
*/

double mag( double input_arr[], int i1, int i2 )
{
	double temp = 0;
	for( int i = i1; i <= i2 ; i++ ) 
	{ 
		temp += pow( input_arr[i], 2 ); 
	}
	return( sqrt( temp ) );
}





/*  Function	: 'sum' 
	Full		: 'inner-element sum'
	Description	:  Returns sum of inner array elements between indices 'i1' and 'i2'
	Rules		:  index parameters 'i1' and 'i2' specify the first and last element considered
	Input Type	:  (double array), (int), (int)
	Return Type :  (double)
*/

double iesum( double input_arr[], int i1, int i2 )
{
	double temp = 0;
	for( int i = i1; i <= i2 ; i++ ) 
	{ 
		temp += input_arr[i]; 
	}
	return( temp );
}





/*  Function	: 'iepro' 
	Full		: 'inner-element multiplication'
	Description	:  Returns product of inner array elements between indices 'i1' and 'i2'
	Rules		:  index parameters 'i1' and 'i2' specify the first and last element considered
	Input Type	:  (double array), (int), (int)
	Return Type :  (double)
*/

double iepro( double input_arr[], int i1, int i2 )
{
	double temp = 0;
	for( int i = i1; i <= i2 ; i++ ) 
	{ 
		temp *= input_arr[i]; 
	}
	return( temp );
}






/*=================================================================================================================================================*/
/* 
Source Name		:	opmatrix.c
Description		:	vector math, inner element operations
Functions  		:	
Structures		: 	vec3x1, mat3x3, vec2arr, arr2vec, addvec, subvec, magvec
Author			:	Brian Cairl
Start Date		:	9/21/2011
*/
/*=================================================================================================================================================*/





/*  Function	: 'mmdot' 
	Full		: 'matrix-matrix dot product'
	Description	:  returns dot product of two 3x3 matrices
	Rules		:  strict inputs
	Input Type	:  (mat3x3), (mat3x3)
	Return Type :  (mat3x3)
*/

mat3x3 mmdot( mat3x3 lhs_mat, mat3x3 rhs_mat ) 
{
	mat3x3 outmat = empty3x3;
	
	outmat.e11 = lhs_mat.e11*rhs_mat.e11 + lhs_mat.e12*rhs_mat.e21 + lhs_mat.e13*rhs_mat.e31;
	outmat.e12 = lhs_mat.e11*rhs_mat.e12 + lhs_mat.e12*rhs_mat.e22 + lhs_mat.e13*rhs_mat.e32;
	outmat.e13 = lhs_mat.e11*rhs_mat.e13 + lhs_mat.e12*rhs_mat.e23 + lhs_mat.e13*rhs_mat.e33;
	
	outmat.e21 = lhs_mat.e21*rhs_mat.e11 + lhs_mat.e22*rhs_mat.e21 + lhs_mat.e23*rhs_mat.e31;
	outmat.e22 = lhs_mat.e21*rhs_mat.e12 + lhs_mat.e22*rhs_mat.e22 + lhs_mat.e23*rhs_mat.e32;
	outmat.e23 = lhs_mat.e21*rhs_mat.e13 + lhs_mat.e22*rhs_mat.e23 + lhs_mat.e23*rhs_mat.e33;
	
	outmat.e31 = lhs_mat.e31*rhs_mat.e11 + lhs_mat.e32*rhs_mat.e21 + lhs_mat.e33*rhs_mat.e31;
	outmat.e32 = lhs_mat.e31*rhs_mat.e12 + lhs_mat.e32*rhs_mat.e22 + lhs_mat.e33*rhs_mat.e32;
	outmat.e33 = lhs_mat.e31*rhs_mat.e13 + lhs_mat.e32*rhs_mat.e23 + lhs_mat.e33*rhs_mat.e33;

	return( outmat );
}






/*  Function	: 'mvdot' 
	Full		: 'matrix-vector dot product'
	Description	:  returns dot product a 3x3 matrices and 3x1 vector
	Rules		:  strict inputs
	Input Type	:  (mat3x3), (vec3x1)
	Return Type :  (mat3x3)
*/

vec3x1 mvdot( mat3x3 lhs_mat, vec3x1 rhs_vec ) 
{
	vec3x1 outvec = empty3x1;
	
	outvec.e11 = lhs_mat.e11*rhs_vec.e11 + lhs_mat.e12*rhs_vec.e21 + lhs_mat.e13*rhs_vec.e31;
	outvec.e21 = lhs_mat.e21*rhs_vec.e11 + lhs_mat.e22*rhs_vec.e21 + lhs_mat.e23*rhs_vec.e31;
	outvec.e31 = lhs_mat.e31*rhs_vec.e11 + lhs_mat.e32*rhs_vec.e21 + lhs_mat.e33*rhs_vec.e31;
	
	return( outvec );
}





/*  Function	: 'vec2arr' 
	Full		: 'vector to array'
	Description	:  converts vector type into 3x1 array
	Rules		:  strict inputs
	Input Type	:  (vec3x1), (floar arr)
	Return Type :  none
*/

void vec2arr( vec3x1 vec, float out_arr[3] ) 
{
	out_arr[0] = vec.e11;
	out_arr[1] = vec.e21;
	out_arr[2] = vec.e31;
}





/*  Function	: 'arr2vec' 
	Full		: 'array to vector'
	Description	:  converts array type into 3x1 vector
	Rules		:  strict inputs
	Input Type	:  (vec3x1)
	Return Type :  (mat3x3)
*/

vec3x1 arr2vec( float in_arr[3] ) 
{
	vec3x1 temp;	
	temp.e11 = in_arr[0];
	temp.e21 = in_arr[1];
	temp.e31 = in_arr[2];
	return( temp );
}





/*  Function	: 'addvec' 
	Full		: 'vector addition'
	Description	:  addition of two 3x1 vectors
	Rules		:  strict inputs
	Input Type	:  (vec3x1),(vec3x1)
	Return Type :  (vec3x1)
*/

vec3x1 addvec( vec3x1 vec1, vec3x1 vec2 ) 
{
	vec3x1 vec_out;	
	vec_out.e11 = vec1.e11 + vec2.e11;
	vec_out.e21 = vec1.e21 + vec2.e21;
	vec_out.e31 = vec1.e31 + vec2.e31;
	return( vec_out );
}





/*  Function	: 'subvec' 
	Full		: 'vector subtraction'
	Description	:  subtraction of two 3x1 vectors
	Rules		:  strict inputs
	Input Type	:  (vec3x1),(vec3x1)
	Return Type :  (vec3x1)
*/

vec3x1 subvec( vec3x1 vec1, vec3x1 vec2 ) 
{
	vec3x1 vec_out;	
	vec_out.e11 = vec2.e11 - vec1.e11;
	vec_out.e21 = vec2.e21 - vec1.e21;
	vec_out.e31 = vec2.e31 - vec1.e31;
	return( vec_out );
}





/*  Function	: 'mulvec' 
	Full		: 'vector multiplication'
	Description	:  multipation of two 3x1 vectors
	Rules		:  strict inputs
	Input Type	:  (vec3x1),(vec3x1)
	Return Type :  (vec3x1)
*/

vec3x1 mulvec( vec3x1 vec1, vec3x1 vec2 ) 
{
	vec3x1 vec_out;	
	vec_out.e11 = vec2.e11*vec1.e11;
	vec_out.e21 = vec2.e21*vec1.e21;
	vec_out.e31 = vec2.e31*vec1.e31;
	return( vec_out );
}





/*  Function	: 'divvec' 
	Full		: 'vector division'
	Description	:  division of two 3x1 vectors
	Rules		:  strict inputs
	Input Type	:  (vec3x1),(vec3x1)
	Return Type :  (vec3x1)
*/

vec3x1 divvec( vec3x1 vec1, vec3x1 vec2 ) 
{
	vec3x1 vec_out;	
	vec_out.e11 = vec2.e11/vec1.e11;
	vec_out.e21 = vec2.e21/vec1.e21;
	vec_out.e31 = vec2.e31/vec1.e31;
	return( vec_out );
}





/*  Function	: 'magvec' 
	Full		: 'vector magnitude'
	Description	:  magnitude of 3x1 vectors
	Rules		:  strict inputs
	Input Type	:  (vec3x1)
	Return Type :  double
*/

double magvec( vec3x1 in_vec ) 
{
	return( sqrt( pow( in_vec.e11, 2 )+pow( in_vec.e21, 2 )+pow( in_vec.e31, 2 ) ) );
}



/*=================================================================================================================================================*/
/* 
Source Name		:	inverse_kinematics.c
Description		:	angular solution function for 4-DOF robotic manipulator given fixed theta-4 position 
Functions  		:	
Author			:	Brian Cairl
Start Date		:	9/21/2011
*/
/*=================================================================================================================================================*/


vec3x1 leg_ik( vec3x1 position, double links[4], double tht3 )
{
	vec3x1 tht = empty3x1;
	
	tht.e11 = atan2( position.e21, position.e11 );
	
	if( ( tht3 > pi ) && ( tht3 < 0 ) )
	{
		tht3 = pi-fabs( tht3 );
	}
	
	vec3x1 l1	= { links[0], 0, 0 };
	mat3x3 R1	= Rz( tht3 );
	
	vec3x1 O4i1	= subvec( position, mvdot( R1, l1 ) );
	
	double R2t4	= sidelaw( links[2], links[3], tht3 );
	double alpa = sign( tht3 )*acos3( R2t4, links[2], links[3] );
	double R1t4 = magvec( O4i1 );
	double beta = acos3( R2t4, links[1], R1t4 );
	
	tht.e21 = pi - ( alpa + beta );
	tht.e31 = -acos3( links[1], R1t4, R2t4 ) + atan2( O4i1.e31, O4i1.e11 );
	
	return( tht );
}
	


// =========================================================================================================================================

int main(void)
{
	//vec3x1 A = empty3x1;
	
	//addvec( A, A );
	DDRC = 0xFF;
	while(1) {
	PORTC = 0xFF;
	delay_ms(50);
	PORTC = 0x00;
	delay_ms(50);}
}
