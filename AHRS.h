#ifndef AHRS_H
#define AHRS_H

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "tm_stm32f4_i2c.h"
#include "USART.h"
#include "tm_stm32f4_delay.h"
#include "ICM_20948.h"
#include "math.h"
#include <stdlib.h>

void UpdateTime(float *deltat, float *lastUpdate);

															
void MadgwickfilterUpdate(float a_x, float a_y, float a_z, 
													float w_x, float w_y, float w_z, 
													float m_x, float m_y, float m_z, 
													float *SEq_1, float *SEq_2, float *SEq_3, float *SEq_4,
													float *b_x, float *b_z,
													float *w_bx, float *w_by, float *w_bz,
													float *deltat, float *lastUpdate);
													
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,
									float *SEq_1, float *SEq_2, float *SEq_3, float *SEq_4,
									float *deltat, float *lastUpdate);

void MahonyQuaternionUpdate(float ax, float ay, float az,
														 float gx, float gy, float gz,
														 float mx, float my, float mz,
														 float *deltat, float *eInt);
	
float Q_rsqrt( float number );
float *getQ(void);
void quat_mult(float *s1, float *s2, float *dest);
void quat_conj(float *p, float *qconj);
float quat_len(float *quat);
void USART1_printfMessage(char *nes);
float roundf(float);
													
#ifndef PI
#define PI 3.14159265358979
#define pi PI
#endif
													
// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (2.0f / 180.0f) // gyroscope measurement error in rad/s (shown as x*(pi/180) deg/s)
#define gyroMeasDrift 3.14159265358979 * (500.5f / 180.0f) // gyroscope measurement error in rad/s/s
//za DLPF 50 Hz je NBW 73.3 Hz prema ICM datasheetu tablica 16. a prema tablici 1 je Noise spectral density 0.015 dps/sqrt(Hz)
//measdrift=sqrt(17.8)*0.015=0.063 --> za DLPF = 11.7 Hz po datasheetu
//measdrift=36 dps izmjereno
//measerror=5 po datasheetu
//measerror=<100 izmjereno
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
#define Ki 0.0f
#define Kp 2.0f * 100.0f
															
#endif
