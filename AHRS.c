#include "AHRS.h"
#include <math.h>


static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float *getQ(void){return q;}


void UpdateTime(float *deltat, float *lastUpdate){
	//char str[20];
	uint32_t Now;
	 Now = TM_DELAY_Time();//vrijednost u ms
  *deltat = 3*((Now - *lastUpdate))/1000.f; //trebamo je u sekundama jer DPS od gyra //*3 je magic number koji daje precizniju vrijednost jer systick presporo broji
  *lastUpdate = Now;
}




void quat_conj(float *p, float *qconj){
    *(qconj+0)=*(p+0);
	*(qconj+1)=-*(p+1);
	*(qconj+2)=-*(p+2);
	*(qconj+3)=-*(p+3);
	return;
}

void quat_mult(float *s1, float *s2, float *dest){
	*(dest+0)= *(s1+0)**(s2+0) - *(s1+1)**(s2+1) - *(s1+2)**(s2+2) - *(s1+3)**(s2+3);
	*(dest+1)= *(s1+0)**(s2+1) + *(s1+1)**(s2+0) + *(s1+2)**(s2+3) - *(s1+3)**(s2+2);
	*(dest+2)= *(s1+0)**(s2+2) - *(s1+1)**(s2+3) + *(s1+2)**(s2+0) + *(s1+3)**(s2+1);
	*(dest+3)= *(s1+0)**(s2+3) + *(s1+1)**(s2+2) - *(s1+2)**(s2+1) + *(s1+3)**(s2+0);
	return;
	}


float quat_len(float *quat){
    float len;
    len=roundf((sqrt(*(quat+0)**(quat+0) + *(quat+1)**(quat+1) + *(quat+2)**(quat+2) + *(quat+3)**(quat+3))));
		/*sprintf(str,"duljina kvaterniona je\n\r %f\n\r", len);
		USART1_printfMessage(str);*/
    return len;
}



float Q_rsqrt( float number ){
//fast inverse square root algoritam od igrice quake 3: Arena
//ima grešku 1.7% sa jednom newtonovom iteracijom
//y=1/sqrt(x)
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}
// Function to compute one filter iteration
void MadgwickfilterUpdate(float a_x, float a_y, float a_z, 
													float w_x, float w_y, float w_z, 
													float m_x, float m_y, float m_z, 
													float *SEq_1, float *SEq_2, float *SEq_3, float *SEq_4,
													float *b_x, float *b_z,
													float *w_bx, float *w_by, float *w_bz,
													float *deltat, float *lastUpdate){
	
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; 																	// objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21,  												// objective function Jacobian elements
				J_32, J_33,
				J_41, J_42, J_43, J_44,
				J_51, J_52, J_53, J_54,
				J_61, J_62, J_63, J_64; 
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; 	// estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z;													  // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; 																				// computed flux in the earth frame
	
	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * *SEq_1;
	float halfSEq_2 = 0.5f * *SEq_2;
	float halfSEq_3 = 0.5f * *SEq_3;
	float halfSEq_4 = 0.5f * *SEq_4;
	float twoSEq_1 = 2.0f * *SEq_1;
	float twoSEq_2 = 2.0f * *SEq_2;
	float twoSEq_3 = 2.0f * *SEq_3;
	float twoSEq_4 = 2.0f * *SEq_4;
	float twob_x = 2.0f * *b_x;
	float twob_z = 2.0f * *b_z;
	float twob_xSEq_1 = 2.0f * *b_x * *SEq_1;
	float twob_xSEq_2 = 2.0f * *b_x * *SEq_2;
	float twob_xSEq_3 = 2.0f * *b_x * *SEq_3;
	float twob_xSEq_4 = 2.0f * *b_x * *SEq_4;
	float twob_zSEq_1 = 2.0f * *b_z * *SEq_1;
	float twob_zSEq_2 = 2.0f * *b_z * *SEq_2;
	float twob_zSEq_3 = 2.0f * *b_z * *SEq_3;
	float twob_zSEq_4 = 2.0f * *b_z * *SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = *SEq_1 * *SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = *SEq_2 * *SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;
	
	// normalise the accelerometer measurement
	
	norm = Q_rsqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x *= norm;
	a_y *= norm;
	a_z *= norm;
	
//	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
//	a_x /= norm;
//	a_y /= norm;
//	a_z /= norm;

	norm = Q_rsqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x *= norm;
	m_y *= norm;
	m_z *= norm;

	// normalise the magnetometer measurement
//	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
//	m_x /= norm;
//	m_y /= norm;
//	m_z /= norm;

	
	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * *SEq_4 - twoSEq_1 * *SEq_3 - a_x;
	f_2 = twoSEq_1 * *SEq_2 + twoSEq_3 * *SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * *SEq_2 - twoSEq_3 * *SEq_3 - a_z;
	f_4 = twob_x * (0.5f - *SEq_3 * *SEq_3 - *SEq_4 * *SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (*SEq_2 * *SEq_3 - *SEq_1 * *SEq_4) + twob_z * (*SEq_1 * *SEq_2 + *SEq_3 * *SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - *SEq_2 * *SEq_2 - *SEq_3 * *SEq_3) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * *SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3; // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;
		
	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
	
	// normalise the gradient to estimate direction of the gyroscope error
//	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
//	SEqHatDot_1 = SEqHatDot_1 / norm;
//	SEqHatDot_2 = SEqHatDot_2 / norm;
//	SEqHatDot_3 = SEqHatDot_3 / norm;
//	SEqHatDot_4 = SEqHatDot_4 / norm;

		norm = Q_rsqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
		SEqHatDot_1 = SEqHatDot_1 * norm;
		SEqHatDot_2 = SEqHatDot_2 * norm;
		SEqHatDot_3 = SEqHatDot_3 * norm;
		SEqHatDot_4 = SEqHatDot_4 * norm;


	
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	
	// compute and remove the gyroscope bsiases
	*w_bx += w_err_x * *deltat * zeta;
	*w_by += w_err_y * *deltat * zeta;
	*w_bz += w_err_z * *deltat * zeta;
	w_x -= *w_bx;
	w_y -= *w_by;
	w_z -= *w_bz;
	
	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 =  halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 =  halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 =  halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	
	// compute then integrate the estimated quaternion rate
	*SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * *deltat;
	*SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * *deltat;
	*SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * *deltat;
	*SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * *deltat;
	
	// normalise quaternion
//	norm = sqrt(*SEq_1 * *SEq_1 + *SEq_2 * *SEq_2 + *SEq_3 * *SEq_3 + *SEq_4 * *SEq_4);
//	*SEq_1 /= norm;
//	*SEq_2 /= norm;
//	*SEq_3 /= norm;
//	*SEq_4 /= norm;

	norm = Q_rsqrt((*SEq_1 * *SEq_1) + *SEq_2 * *SEq_2 + *SEq_3 * *SEq_3 + *SEq_4 * *SEq_4);
	*SEq_1 *= norm;
	*SEq_2 *= norm;
	*SEq_3 *= norm;
	*SEq_4 *= norm;
	
	// compute flux in the earth frame
	SEq_1SEq_2 = *SEq_1 * *SEq_2; // recompute axulirary variables
	SEq_1SEq_3 = *SEq_1 * *SEq_3;
	SEq_1SEq_4 = *SEq_1 * *SEq_4;
	SEq_3SEq_4 = *SEq_3 * *SEq_4;
	SEq_2SEq_3 = *SEq_2 * *SEq_3;
	SEq_2SEq_4 = *SEq_2 * *SEq_4;
	h_x = twom_x * (0.5f - *SEq_3 * *SEq_3 - *SEq_4 * *SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - *SEq_2 * *SEq_2 - *SEq_4 * *SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - *SEq_2 * *SEq_2 - *SEq_3 * *SEq_3);
	
	//ovo se da izbacit ali onda main treba izmjenit da cita SEq_x vrijednosti
	q[0]=*SEq_1;
	q[1]=*SEq_2;
	q[2]=*SEq_3;
	q[3]=*SEq_4;
	
	
	// normalise the flux vector to have only components in the x and z
	*b_x = sqrt((h_x * h_x) + (h_y * h_y));
	*b_z = h_z;
}
		


// Global system variables
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,
									float *SEq_1, float *SEq_2, float *SEq_3, float *SEq_4,
									float *deltat, float *lastUpdate) {
										
  // Local system variables
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
  float f_1, f_2, f_3; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  // Axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * *SEq_1;
  float halfSEq_2 = 0.5f * *SEq_2;
  float halfSEq_3 = 0.5f * *SEq_3;
  float halfSEq_4 = 0.5f * *SEq_4;
  float twoSEq_1 = 2.0f * *SEq_1;
  float twoSEq_2 = 2.0f * *SEq_2;
  float twoSEq_3 = 2.0f * *SEq_3;

  // Normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * *SEq_4 - twoSEq_1 * *SEq_3 - a_x;
  f_2 = twoSEq_1 * *SEq_2 + twoSEq_3 * *SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * *SEq_2 - twoSEq_3 * *SEq_3 - a_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * *SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;
  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // Compute then integrate the estimated quaternion derrivative
  *SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * *deltat;
  *SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * *deltat;
  *SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * *deltat;
  *SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * *deltat;
  // Normalise quaternion
  norm = sqrt(*SEq_1 * *SEq_1 + *SEq_2 * *SEq_2 + *SEq_3 * *SEq_3 + *SEq_4 * *SEq_4);
  *SEq_1 /= norm;
  *SEq_2 /= norm;
  *SEq_3 /= norm;
  *SEq_4 /= norm;
	
	q[0]=*SEq_1;
	q[1]=*SEq_2;
	q[2]=*SEq_3;
	q[3]=*SEq_4;
 }
									
 void MahonyQuaternionUpdate(float ax, float ay, float az,
														 float gx, float gy, float gz,
														 float mx, float my, float mz,
														 float *deltat, float *eInt){
															 
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    *(eInt+0) += ex;      // accumulate integral error
    *(eInt+1) += ey;
    *(eInt+2) += ez;
  }
  else
  {
    *(eInt+0) = 0.0f;     // prevent integral wind up
    *(eInt+1) = 0.0f;
    *(eInt+2) = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];
 
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * *deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * *deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * *deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * *deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
 

