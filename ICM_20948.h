
/**/
#ifndef ICM_20948_H
#define ICM_20948_H

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "tm_stm32f4_i2c.h"
#include "USART.h"
#include <stdio.h>
#include <stdlib.h>
//#include "tm_stm32f4_gpio.h"

void USART1_SendNumber(int32_t x);
void USART1_DecToHexSend(int16_t x);
void USART1_SendFloat(float x);
void quat_mult(float *s1, float *s2, float *dest);
void quat_conj(float *source, float *dest);
#define PI 3.14159265358979
#define pi PI



/*-------------REGISTER ADDRESSES defines----------------*/

/*address of ICM_20948*/
#define ICM_20948_BASE 	((uint8_t)0xD0)
#define ICM ICM_20948_BASE

#define AK_09916					((uint8_t)0x18)
#define AK AK_09916

#define REG_BANK_SEL 		((uint8_t)0x7F)
#define BANK_SEL REG_BANK_SEL

#define USER_CTRL 		((uint8_t)0x03)

#define INT_ENABLE 		((uint8_t)0x10)
#define INT_ENABLE_1 	((uint8_t)0x11)

#define INT_STATUS_1	((uint8_t)0x1A)

/*fifo registers*/
#define FIFO_EN_1 	((uint8_t)0x66)
#define FIFO_EN_2 	((uint8_t)0x67)
#define FIFO_RST	 	((uint8_t)0x68)
#define FIFO_MODE	 	((uint8_t)0x69)
#define FIFO_COUNTH ((uint8_t)0x70)
#define FIFO_COUNTL ((uint8_t)0x71)
#define FIFO_R_W		((uint8_t)0x72)


/*select user register bank */
#define REG_BANK_0                    ((uint8_t)0x00)
#define REG_BANK_1           				  ((uint8_t)0x10) 
#define REG_BANK_2             			  ((uint8_t)0x20)
#define REG_BANK_3                    ((uint8_t)0x30)
#define REG_BANK(BANK)		(((BANK) == REG_BANK_0) || \
													 ((BANK) == REG_BANK_1) || \
													 ((BANK) == REG_BANK_2) || \
													 ((BANK) == REG_BANK_3))
													 
/*select power management register*/
#define REG_PWR_MGMT_1             			  ((uint8_t)0x06)
#define REG_PWR_MGMT_2                    ((uint8_t)0x07)
#define REG_PWR_MGMT (MGMT)		(((MGMT) == REG_PWR_MGMT_1) || \
															 ((MGMT) == REG_PWR_MGMT_2))

/*address of who am i register and its contet*/
#define WAI_ICM					((uint8_t)0x00)
#define WAI_AK					((uint8_t)0x01)
#define ICM20948_ID 		((uint8_t)0xEA)
#define AK09916_ID			((uint8_t)0x09)

/*user control registers*/
#define REG_USER_CTRL			((uint8_t)0x03)

/*interrupt pin configuration register*/
#define INT_PIN_CFG 	 	((uint8_t) 0x0F)

/*acceleerometer registers*/
#define ACCEL_CONFIG_1	((uint8_t)0x14)
#define ACCEL_CONFIG_2 	((uint8_t)0x15)

#define ACCEL_XOUT_H 		((uint8_t)0x2D)
#define ACCEL_XOUT_L 		((uint8_t)0x2E)
#define ACCEL_YOUT_H 		((uint8_t)0x2F)
#define ACCEL_YOUT_L 		((uint8_t)0x30)
#define ACCEL_ZOUT_H 		((uint8_t)0x31)
#define ACCEL_ZOUT_L 		((uint8_t)0x32)

#define XA_OFFSET_H			((uint8_t)0x14)
#define XA_OFFSET_L			((uint8_t)0x15)
#define YA_OFFSET_H			((uint8_t)0x17)
#define YA_OFFSET_L			((uint8_t)0x18)
#define ZA_OFFSET_H			((uint8_t)0x1A)
#define ZA_OFFSET_L			((uint8_t)0x1B)

#define SELF_TEST_X_ACCEL ((uint8_t)0x0E)
#define SELF_TEST_Y_ACCEL ((uint8_t)0x0F)
#define SELF_TEST_Z_ACCEL ((uint8_t)0x10)


#define ACCEL_SMPLRT_DIV_1 ((uint8_t)0x10)
#define ACCEL_SMPLRT_DIV_2 ((uint8_t)0x11)
#define ACCEL_SMPLRT_LSB ACCEL_SMPLRT_DIV_1
#define ACCEL_SMPLRT_MSB ACCEL_SMPLRT_DIV_2

#define LP_CONFIG ((uint8_t)0x05)


/*gyro registers*/
#define GYRO_CONFIG_1 	((uint8_t)0x01)
#define GYRO_CONFIG_2 	((uint8_t)0x02)

#define GYRO_XOUT_H 		((uint8_t)0x33)
#define GYRO_XOUT_L 		((uint8_t)0x34)
#define GYRO_YOUT_H 		((uint8_t)0x35)
#define GYRO_YOUT_L 		((uint8_t)0x36)
#define GYRO_ZOUT_H 		((uint8_t)0x37)
#define GYRO_ZOUT_L 		((uint8_t)0x38)

#define XG_OFFS_USRH		((uint8_t)0x03)
#define XG_OFFS_USRL		((uint8_t)0x04)	
#define YG_OFFS_USRH		((uint8_t)0x05)	
#define YG_OFFS_USRL		((uint8_t)0x06)	
#define ZG_OFFS_USRH		((uint8_t)0x07)	
#define ZG_OFFS_USRL		((uint8_t)0x08) 	

#define SELF_TEST_X_GYRO ((uint8_t)0x02)
#define SELF_TEST_Y_GYRO ((uint8_t)0x03)
#define SELF_TEST_Z_GYRO ((uint8_t)0x04)

#define GYRO_SMPLRT_DIV	((uint8_t)0x00)

/*magnetometer registers*/

#define WIA2			((uint8_t)0x01)
#define ST1				((uint8_t)0x10)
#define HXL				((uint8_t)0x11)
#define HXH				((uint8_t)0x12)
#define HYL				((uint8_t)0x13)
#define HYH				((uint8_t)0x14)
#define HZL				((uint8_t)0x15)
#define HZH				((uint8_t)0x16)
#define ST2				((uint8_t)0x18)
#define CNTL2			((uint8_t)0x31)
#define CNTL3			((uint8_t)0x32)
#define TS1				((uint8_t)0x33)
#define TS2				((uint8_t)0x34)


/*---------------------------------------------------*/

/*-----------CONTROL REGISTER VALUES-----------------*/

/*user control register*/
		/*enable or disable DMP*/
		#define DMP_EN                    				((uint8_t)1<<7)
		#define DMP_DIS           				  			((uint8_t)0x00)  
		#define IS_DMP_ENABLED(DMP_ENABLED)	 (((DMP_ENABLED) == DMP_EN) || \
																					((DMP_ENABLED) == DMP_DIS))																					
		/*enable or disable FIFO*/
		#define FIFO_EN 	                  		 	((uint8_t)1<<6)
		#define FIFO_DIS          				  			((uint8_t)0<<6)  
		#define IS_FIFO_(FIFO_ENABLED)	 (((FIFO_ENABLED) == FIFO_EN) || \
																			((FIFO_ENABLED) == FIFO_DIS))

		#define I2C_MST_EN		 				((uint8_t)1<<5) // Enable the I2C Master I/F module, 0 ako ocu bypassat vanjski senzor na ukontroler, vidi BYPASS_EN
		#define I2C_MST_DIS						((uint8_t)0<<5)
		#define	I2C_if_EN 						((uint8_t)1<<4) //ovo mos izbrisat ali se sjeti toga kod inicijalizacije
		#define	I2C_if_DIS 						((uint8_t)0<<4)
		#define	DMP_RST 							((uint8_t)1<<3)
		#define	SRAM_RST							((uint8_t)1<<2)
		#define	MST_RST								((uint8_t)1<<1)
		/*ovo :1 oznacava da ces koristit samo 1 bit od cijelog uint8_t
		Odma se poravnavaju desno.
		vidi na https://www.tutorialspoint.com/cprogramming/c_bit_fields.htm
		te sve varijable treba shiftat na svoja mjesta
		ovisno o njihovoj poziciji u user_ctrl registru*/


/*power management1 register control values*/
		#define DEVICE_RESET                    	((uint8_t)1<<7)
		#define SLEEP		          				  			((uint8_t)1<<6)
		#define NOSLEEP	          				  			((uint8_t)0<<6)
		#define LP_EN		          				  			((uint8_t)1<<5)
		#define LP_DIS	          				  			((uint8_t)0<<5)
		#define TEMP_DIS          				  			((uint8_t)1<<3)

		#define INTERNAL_20MHz     				  			((uint8_t)0<<0)
		#define AUTO_SEL          				  			((uint8_t)1<<1)
		#define STOP_CLK          				  			((uint8_t)3<<0)
		#define CLKSEL(CLKSEL)	 						 (((CLKSEL) == INTERNAL_20MHz) || \
																					((CLKSEL) == AUTO_SEL) || \
																					((CLKSEL) == STOP_CLK))
/*power management2 register control values*/
		#define ENABLE_ACCEL					((uint8_t)0x07)
		#define ENABLE_GYRO						((uint8_t)0x38)
		#define ENABLE_GYRO_ACCEL			((uint8_t)0x00)
		#define DISABLE_GYRO_ACCEL		((uint8_t)0x3F)
	
/*accelerometer configuration register control*/

		#define ACCEL_ST_ENABLE_ALL_AXES		((uint8_t)0x1C)
		#define ACCEL_ST_DISABLE_ALL_AXES		((uint8_t)0x00)
		
		#define DLPF_ACC_250HZ 				((uint8_t)0x01)
		#define DLPF_ACC_111HZ 				((uint8_t)0x02) // 111.4 kHz
		#define DLPF_ACC_50HZ 				((uint8_t)0x03)
		#define DLPF_ACC_24HZ 				((uint8_t)0x04)
		#define DLPF_ACC_12HZ 				((uint8_t)0x05)
		#define DLPF_ACC_6HZ 					((uint8_t)0x06)
		#define DLPF_ACC_473HZ 				((uint8_t)0x07)

	typedef enum{
			ACCEL_RANGE_2G = 0,
			ACCEL_RANGE_4G,
			ACCEL_RANGE_8G,
			ACCEL_RANGE_16G
		}accel_range_t;

		#define ACCEL_AVG_1X 					((uint8_t)0x00)
		#define ACCEL_AVG_4X 					((uint8_t)0x00)
		#define ACCEL_AVG_8X 					((uint8_t)0x01)
		#define ACCEL_AVG_16X 				((uint8_t)0x02)
		#define ACCEL_AVG_32X 				((uint8_t)0x03)
		
		//1125/(1+ACCEL_SMPLRT_DIV)
		#define ACCEL_SMPLRT_1kHz			((uint16_t)0x000)
		#define ACCEL_SMPLRT_563Hz		((uint16_t)0x001)
		#define ACCEL_SMPLRT_4Hz			((uint16_t)0xFFF)
		#define ACCEL_SMPLRT_14Hz			((uint16_t)0x00E)
		#define ACCEL_SMPLRT_25Hz			((uint16_t)0x02C)
		
/*gyro configuration register control */
		
		#define GYRO_ST_ENABLE_ALL_AXES		((uint8_t)0x38)
		#define GYRO_ST_DISABLE_ALL_AXES	((uint8_t)0x00)
		
		#define DLPF_GYRO_200HZ 			((uint8_t)0x00)
		#define DLPF_GYRO_150HZ 			((uint8_t)0x01)
		#define DLPF_GYRO_120HZ 			((uint8_t)0x02) //119.5 Hz
		#define DLPF_GYRO_50HZ 				((uint8_t)0x03)
		#define DLPF_GYRO_24HZ 				((uint8_t)0x04)
		#define DLPF_GYRO_12HZ 				((uint8_t)0x05)
		#define DLPF_GYRO_6HZ 				((uint8_t)0x06)
		#define DLPF_GYRO_360HZ 			((uint8_t)0x07)
		
		#define GYRO_CYCLE						((uint8_t)1<<4)

	typedef enum{
			GYRO_RANGE_250DPS = 0,
			GYRO_RANGE_500DPS = 0x01,
			GYRO_RANGE_1000DPS = 0x10,
			GYRO_RANGE_2000DPS = 0x11
		}gyro_range_t;
		
		//1.1khZ/(1+GYRO_SMPLRT_DIV[7:0])
		#define GYRO_SMPLRT_1kHz			((uint8_t)0x00)
		#define GYRO_SMPLRT_550Hz			((uint8_t)0x01)
		#define GYRO_SMPLRT_367Hz			((uint8_t)0x02)
		#define GYRO_SMPLRT_275Hz			((uint8_t)0x03)
		#define GYRO_SMPLRT_220Hz			((uint8_t)0x04)
		#define GYRO_SMPLRT_183Hz			((uint8_t)0x05)		
		#define GYRO_SMPLRT_4Hz				((uint8_t)0xFF)
		#define GYRO_SMPLRT_25Hz			((uint8_t)0x2B)
		
		
		#define GYRO_AVG_1X 					((uint8_t)0x00)
		#define GYRO_AVG_2X 					((uint8_t)0x01)
		#define GYRO_AVG_4X 					((uint8_t)0x02)
		#define GYRO_AVG_8X 					((uint8_t)0x03)
		#define GYRO_AVG_16X 					((uint8_t)0x04)
		#define GYRO_AVG_32X 					((uint8_t)0x05)
		#define GYRO_AVG_64X 					((uint8_t)0x06)
		#define GYRO_AVG_128X 				((uint8_t)0x07)


/*magnetometer configuration register control */

		#define MAG_POWER_DOWN_MODE  	((uint8_t)0x00)
		#define MAG_SINGLE_MODE 		 	((uint8_t)0x01)
		#define MAG_CONTINUOUS_MODE1 	((uint8_t)0x02)
		#define MAG_CONTINUOUS_MODE2 	((uint8_t)0x04)
		#define MAG_CONTINUOUS_MODE3 	((uint8_t)0x06)
		#define MAG_CONTINUOUS_MODE4 	((uint8_t)0x08)
		#define MAG_SELFTEST_MODE		 	((uint8_t)0x10)
		
		#define MAG_10Hz	MAG_CONTINUOUS_MODE1
		#define MAG_20Hz	MAG_CONTINUOUS_MODE2
		#define MAG_50Hz	MAG_CONTINUOUS_MODE3
		#define MAG_100Hz	MAG_CONTINUOUS_MODE4
		
		#define MAG_RESET 			((uint8_t)0x01)

/*internal pin configuration*/
		#define BYPASS_EN 			((uint8_t)0x02)
		
/*fifo configuration register control*/		
		#define FIFO_ASSERT 				((uint8_t)0x1F)
		#define FIFO_DEASSERT 			((uint8_t)0x00)

		#define FIFO_GYRO_ACCEL_EN	((uint8_t)0x1E)
		#define FIFO_GYRO_EN				((uint8_t)0x0E)
		#define FIFO_ACCEL_EN				((uint8_t)0x10)
		#define FIFO_TEMP_EN				((uint8_t)0x01)

		#define FIFO_SNAPSHOT				((uint8_t)0x1F)
		#define FIFO_STREAM					((uint8_t)0x00)
		
/*other*/
		#define DISABLE_ALL 		((uint8_t)0x00)
		#define ODR_ALIGN_EN		((uint8_t)0x09)
		
/*---------------------------------------------------*/
/*------------STRUCTURES AND OTHER STUFF-------------*/

													 
/*Bank modes*/
typedef enum {
	ICM_UserBank_0=0,
	ICM_UserBank_1=1,      
	ICM_UserBank_2=2,      
	ICM_UserBank_3=3,     
} ICM_UserBank_t;

/*Power management register*/
typedef enum {
	ICM_PwrMgmt_1,
	ICM_PwrMgmt_2,      
} ICM_PwrMgmt_t;

typedef enum {
	ICM_Accel,
	ICM_Gyro,
	ICM_Magn,
} ICM_sensor_t;

typedef struct{
	uint8_t accel_DLPF;
	accel_range_t accel_range;
	uint8_t accel_averaging;
	uint8_t accel_DLPFenable;
	uint16_t accel_smplrate;
}AccConfig_t;

typedef struct{
	uint8_t gyro_DLPF;
	gyro_range_t gyro_range;
	uint8_t gyro_averaging;
	uint8_t gyro_DLPFenable;
	uint8_t gyro_smplrate;
}GyroConfig_t;



/*------------------FUNCTIONS-----------------------*/

/* 
 * opis: odaberi set registara koji zelis koristit u ICM20948
 * param: koj I2C	se koristi, vrijednost I2C1-I2C3
 * param: koj userbank, vrijednost od ICM_UserBank_t
 * return void
*/
void ID_SelectUserBank(I2C_TypeDef* I2Cx, ICM_UserBank_t UserBank);

/* 
 * opis: za ICM20948 vraca 0xE7 vrijednost
 * vraca 1 ako je to taj, 0 ako nije
 * param1: koj I2C je spojen
 * param2: adresa uredaja kojeg pitas za ime
*/
void ID_WhoAmI(I2C_TypeDef* I2Cx, uint8_t address);

/*
 *opis: odaberi koj power configuration registar zelis namjestit.
 *unutar ove funkcije namjesti detalja
 *param1: na koji i2c je spojeno
 *param2: koj power config
*/
void	ID_ConfigPowerManagement(I2C_TypeDef* I2Cx, ICM_PwrMgmt_t PwrMgmt);
/* 
* opis: inicijalizacija user_control registra
* param1: jel zelis DMP - 1 enable, 0 disable
* param2: jel zelis FIFO - 1 enable, 0 disable
* ret: void 
*/ 

//////////*static void ID_ICM_20948_INT_user_control(uint8_t DMP, uint8_t FIFO);*/

/*
 * opis: inicijalizacija ICM20948 senzora
 * param: na koji I2C je spojen
*/ 
void ID_ICM20948_Init(I2C_TypeDef* I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig);

/*
 * opis: cita raw sensor data
 * param1: na koji I2C je spojen senzor
 * param2: sa koje senzora citaš vrijednost. struktura ICM_sensor_t; sadrzi moguce senzore
 * param3: u koje polje zelis spremit vrijednosti
*/
void ID_ReadSensorData(I2C_TypeDef* I2Cx, ICM_sensor_t sensor, int16_t* data);

/*
 * opis: konfigurira gyro
 * param1: na koji i2c je spojen
 * param2: LP filter frequency
 * param3: osjetljivost (gyro full scale select)
 * param4: averaging filter velicina
 * param5: jel zelis Filter (Filter choice)
 * param6: output sample rate ziroskopa
*/
void ID_GyroConfig(I2C_TypeDef* I2Cx, GyroConfig_t *GyroConfig);

/*
 * opis: konfigurira akcelerometar
 * param1: na koji i2c je spojen
 * param2: LP filter frequency
 * param3: osjetljivost (accel full scale select)
 * param4: averaging filter velicina
 * param5: jel zelis Filter (Filter choice)
 * param6: output sample rate akcelerometra
*/
void ID_AccConfig(I2C_TypeDef* I2Cx, AccConfig_t *AccConfig);
/*
 * opis: konfigurira magnetometar
 * param1: na koji i2c je spojen
 * param2: biraj mode u kojem ce radit 
*/
void ID_MagnConfig(I2C_TypeDef* I2Cx, uint8_t magmode);

/*
 * opis: zaustavi magnetometar
 * param1: na koji i2c je spojen
*/
void ID_StopMag(I2C_TypeDef* I2Cx);

/*
 * opis: napravi samodijagnozu magnetometra
 * param1: na koji i2c je spojen
 * return value: '1' if self test successful, '0' otherwise
*/
uint8_t ID_MagSelftest(I2C_TypeDef* I2Cx);

/*
 * opis: vraca rezoluciju senzora
 * return value: rezolucija senzora
*/
float ID_getAccelRes(void);
float ID_getGyroRes(void);
float ID_getMagnRes(void);

/*
* opis: enable/disable self test pojedinih senzora
* param1: na koj I2C	je spojen
* param2: za koje senzore zelis enbleat, val: ENABLE_GYRO_ACCEL i ostali
*/
void ID_EnableSelfTest(I2C_TypeDef* I2Cx, uint8_t sensor);
void ID_DisableSelfTest(I2C_TypeDef* I2Cx, uint8_t sensor);

/*
* opis: vraca postotak devijacije u odnosu na factory trim vrijednosti
* param1: i2c na koji je spojen senzor
* param2: ne postoji ali moze se dodat, (float *destination) -> vrijednost polja
* u koje se spremaju vrijednosti. onda treba maknut iz funkcije float destination.
*/
void ID_ICM20948SelfTest(I2C_TypeDef* I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig);

/*
* opis: Reset FIFO
* param0: i2c na koju je spojen senzor
*/
void ID_FIFOReset(I2C_TypeDef* I2Cx);

/*
* opis: Disable all interupts, disable fifo and turn on internal clock
* param0: i2c na koju je spojen senzor
*/
void ID_DisableAllInterrupts(I2C_TypeDef* I2Cx);

/*
* opis: vraca gyro i accel bias
* param0: i2c na koju je spojen senzor
* Accel Bias i Gyro Bias su spremljeni u globalne varijable u ICM_20948.c gyroBias[3] i accelBias[3]
*/
void ID_ICM20948Calibration(I2C_TypeDef * I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig, float *x, float *y, float *z, int32_t *gyro_bias, int32_t *accel_bias);
/*
* opis: kalibracija magnetometra na soft i hard iron pogreske
* param0: i2c na koju je spojen senzor
* return: 1 ili 0 ovisno je li uspjelo
*/
uint8_t ID_MagnCalibration(I2C_TypeDef * I2Cx, float *x, float *y, float *z, float *kappa, float *sigma, uint16_t *sample_count, float  *qkoso, float *qzos, float *mag_bias) ;
/*
*opis: ispravlja izmjerenu sirovu vrijednost za soft iron
*param: raw data s magnetometra*
*/
void ID_MagnCorrection(int16_t * MagMeasurement, int debug, float kappa, float sigma, float  qkoso[4], float qzos[4], float mag_bias[3]);
void test(float * x, float *y, float *z);
void f(float *kappa, float *sigma, float *qkoso, float *qzos, float *mag_bias);
void AccGyroParam(GyroConfig_t *GyroConfig, AccConfig_t *AccConfig);
uint8_t ID_MagDataReady(I2C_TypeDef *I2Cx);
#endif
