#include <stm32f4xx.h> // common stuff
#include <stm32f4xx_gpio.h> // gpio control
#include <stm32f4xx_rcc.h> // reset and clocking
#include <main.h>


float x[1500], y[1500], z[1500];

int main(void)
{	
	extern char str[50];
	
	float b_x = 1, b_z = 0; // reference direction of flux in earth frame
	float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;
	float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
	float deltat=0, lastUpdate=0;
	float eInt[3];
	
	volatile float ax, ay, az, gx=0, gy=0, gz=0, mx, my, mz; //vrijednosti senzora
	volatile float pitch, yaw, roll;
	extern float magScale[3];
	//float kappa, sigma, mag_bias[3], qkoso[4], qzos[4];
	extern uint32_t sumCount, sum;
	uint8_t debug=0;
	//int32_t gyro_bias[3], accel_bias[3];
	int32_t gyro_bias[3]={ -3803, -11048, 621}, accel_bias[3]={243, -197, 1310}; //-3803, -11048, 621
	float sigma=0.939271, kappa = 0.895753, mag_bias[3]={-202, -83, 116},
				qkoso[4]={0.805547, -0.557601, -0.191824, 0.000000},
				qzos[4]={ 0.795827, 0.000000, 0.000000, -0.602725};
	 
	AccConfig_t AccConfig;
	GyroConfig_t GyroConfig;
	
	int16_t AccelRawData[3], GyroRawData[3], MagRawData[3];
	uint8_t ICM_found=0, AK_found=0;
	uint16_t i, sample_count;
	char c;
	
	/* USART1 init PB6 (TX) PB7(RX)*/
	USART1_Init();
	/* Initialize system */
	SystemInit();
	/* Initialize delay */
	TM_DELAY_Init();
	TM_I2C_Init(ID_I2C, TM_I2C_PinsPack_1, 115200);
	Delayms(500);
//	USART1_printfMessage("\n\rInitializing ICM20948...\n\r");
	ID_ICM20948_Init(ID_I2C, &GyroConfig, &AccConfig); //initialize ICM and do Magnetometer self-test

/*-------pretrazi jel kaj spojeno---------*/	
	

// iterate adresses from 0x02 to 0xFF.
// infinite loop

	do{
		for (i = 2; i < 0xFF; i += 2) {
			if(i==0xFE && (AK_found==0 && ICM_found==0))USART1_printfMessage("If nothing is connected, restart power for uC\n\r");
			if(TM_I2C_IsDeviceConnected(ID_I2C, i)){
					USART1_printfMessage("\n\rsomething on adress ");
					USART1_DecToHexSend(i);
				if(TM_I2C_Read(ID_I2C, i, 1)==0x09){
					AK_found=1;
					USART1_printfMessage("\n\ra it's AK09916 with ID 0x09");
					USART1_printfMessage("\n\r");
				}
				if(TM_I2C_Read(ID_I2C, i, 0)==0xEA){
					ICM_found=1;
					USART1_printfMessage("\n\ra it's ICM20948 with ID 0xEA");
					USART1_printfMessage("\n\r");
				}
			}
			if(!(AK_found && ICM_found) && i==0xFE)USART1_printfMessage("\n\rNothin connected, searching again\n\r");
		}
	}while(!(ICM_found && AK_found));
	USART1_printfMessage("\n\rI will do self test, gyro and accelerometer calibration now"
												"\n\r position ICM horizontally with it's x+ axe towards north"
												"\n\rpress y to continue");
	while(!(c=='y' || c=='Y' || c=='z' || c=='Z'))(USART1_Dequeue(&c));
	c=0;
	
	
	ID_ICM20948SelfTest(ID_I2C, &GyroConfig, &AccConfig);
	ID_ICM20948Calibration(ID_I2C, &GyroConfig, &AccConfig, x,y,z, gyro_bias, accel_bias);

	
	
	while(!(ID_MagSelftest(ID_I2C))); //must be done before magn ID_MagnConfig
	ID_ICM20948_Init(ID_I2C, &GyroConfig, &AccConfig); 
	USART1_printfMessage("\n\rCalibration of magnetometer for Soft and Hard iron");	
	while(!(ID_MagnCalibration(ID_I2C,x,y,z, &kappa, &sigma, &sample_count, qkoso, qzos, mag_bias)));	//IDMagnCalibration must be after ICMInit


	ID_ICM20948_Init(ID_I2C, &GyroConfig, &AccConfig);
	USART1_printfMessage("\n\rabout to print sensor orientation\n\rpress y to continue\n\r");
	while(!(c=='y'))(USART1_Dequeue(&c));
	c=0;
	

	switch (GyroConfig.gyro_range) 
  {
  	case(GYRO_RANGE_250DPS):
					gyro_bias[0] /= 1.f;
					gyro_bias[1] /= 1.f;
					gyro_bias[2] /= 1.f;
  		break;
  	case(GYRO_RANGE_500DPS):	
					gyro_bias[0] /= 2.f;
					gyro_bias[1] /= 2.f;
					gyro_bias[2] /= 2.f;
  		break;
		case(GYRO_RANGE_1000DPS):
					gyro_bias[0] /= 4.f;
					gyro_bias[1] /= 4.f;
					gyro_bias[2] /= 4.f;
  		break;
		case(GYRO_RANGE_2000DPS):
					gyro_bias[0] /= 8.f;
					gyro_bias[1] /= 8.f;
					gyro_bias[2] /= 8.f;
  		break;
  }
	
	switch (AccConfig.accel_range) //magic numbers
  {
  	case(ACCEL_RANGE_2G):
					accel_bias[0] /= 1.f;
					accel_bias[1] /= 1.f;
					accel_bias[2] /= 1.f;
  		break;
  	case(ACCEL_RANGE_4G):	
					accel_bias[0] /= 2.f;
					accel_bias[1] /= 2.f;
					accel_bias[2] /= 2.f;
  		break;
		case(ACCEL_RANGE_8G):
					accel_bias[0] /= 4.f;
					accel_bias[1] /= 4.f;
					accel_bias[2] /= 4.f;
  		break;
		case(ACCEL_RANGE_16G):
					accel_bias[0] /= 8.f;
					accel_bias[1] /= 8.f;
					accel_bias[2] /= 8.f;
  		break;
  }
				
	i=0;
	UpdateTime(&deltat, &lastUpdate);
	while(1){



		ID_ReadSensorData(ID_I2C, ICM_Accel, AccelRawData);
		ax = ((float)AccelRawData[0]-(float)accel_bias[0])*ID_getAccelRes();
		ay = ((float)AccelRawData[1]-(float)accel_bias[1])*ID_getAccelRes();
		az = ((float)AccelRawData[2]-(float)accel_bias[2])*ID_getAccelRes();
//uncomment one of sprintf-s and USART1 depending on which type of print do you want	
//		sprintf(str, "%f,%f,%f\n", ax, ay, az); //python
//		sprintf(str, "%f %f %f\n", ax, ay, az);//matlab
		sprintf(str, "\r%10.6f %10.6f %10.6f", ax, ay, az);
//		USART1_printfMessage(str);
		
		ID_ReadSensorData(ID_I2C, ICM_Gyro, GyroRawData);
		gx = ((float)GyroRawData[0]-(float)gyro_bias[0])*ID_getGyroRes();
		gy = ((float)GyroRawData[1]-(float)gyro_bias[1])*ID_getGyroRes();
		gz = ((float)GyroRawData[2]-(float)gyro_bias[2])*ID_getGyroRes();
//		sprintf(str, "%f %f %f\n", gx, gy, gz);//matlab
//		sprintf(str, "%f,%f,%f", gx, gy, gz);//python
		sprintf(str, "\r%13.6f %13.6f %13.6f", gx, gy, gz);
	//	USART1_printfMessage(str);

		if(ID_MagDataReady(ID_I2C)){
			ID_ReadSensorData(ID_I2C, ICM_Magn, MagRawData);
			ID_MagnCorrection(MagRawData, debug, kappa, sigma, qkoso, qzos, mag_bias); 
			//Xcorrect=(X-b)*A; b - hard iron, A - soft iron
			mx = ((float)MagRawData[0]*ID_getMagnRes())*10.f; //*10 za miliGause
			my = ((float)MagRawData[1]*ID_getMagnRes())*10.f;
			mz = ((float)MagRawData[2]*ID_getMagnRes())*10.f;
	//	sprintf(str, "%f\n%f\n%f\n", mx, my, mz);
	//		USART1_printfMessage(str);
		}

		//start madgwick filter only if sensor data is different then previous data, otherwise skip
	if(!((x[0]==ax || x[0]==ay || x[0]==az) &&
			 (x[3]==gx || x[4]==gy || x[5]==gz))){
		x[0] = ax;
		x[1] = ay;
		x[2] = az;
		x[3] = gx;
		x[4] = gy;
		x[5] = gz;
		i++;
		UpdateTime(&deltat, &lastUpdate); //calculate time spent from last Madgwick itteration
/*		sprintf(str,"\n\r %d. je %10.6f %10.6f %10.6f\t"
													"%13.6f %13.6f %13.6f\t\t%5.3f",
												i, ax, ay, az, gx, gy, gz, deltat);
		USART1_printfMessage(str);
*/
				 //pozovi algoritam samo ako imaš nove podatke
				
		
			 
		MadgwickfilterUpdate(1000*ax, 1000*ay, 1000*az,
												 gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
												 my, mx, mz,
												 &SEq_1, &SEq_2, &SEq_3, &SEq_4,
												 &b_x, &b_z,
												 &w_bx, &w_by, &w_bz,
												 &deltat, &lastUpdate);
		
		/*filterUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
								 1000*ax, 1000*ay, 1000*az,
								 &SEq_1, &SEq_2, &SEq_3, &SEq_4,
								 &deltat, &lastUpdate);	
	
		
		MahonyQuaternionUpdate(ax, ay, az,
														 gx, gy, gz,
														 mx, my, mz,
														 &deltat, eInt);
			*/
		}
			 
		if(debug){
				USART1_printfMessage("\n\r ax = "); USART1_SendFloat((float)(1000.0f * ax)); USART1_printfMessage(" mg\n\r");
				USART1_printfMessage(" ay = "); USART1_SendFloat((float)(1000.0f * ay)); USART1_printfMessage(" mg\n\r");
				USART1_printfMessage(" az = "); USART1_SendFloat((float)(1000.0f * az)); USART1_printfMessage(" mg\n\r");				
				
				USART1_printfMessage(" gx = "); USART1_SendFloat(gx); USART1_printfMessage(" DPS\n\r");
				USART1_printfMessage(" gy = "); USART1_SendFloat(gy); USART1_printfMessage(" DPS\n\r");
				USART1_printfMessage(" gz = "); USART1_SendFloat(gz); USART1_printfMessage(" DPS\n\r");
				
				USART1_printfMessage(" mx = "); USART1_SendFloat(mx); USART1_printfMessage(" uT\n\r");
				USART1_printfMessage(" my = "); USART1_SendFloat(my); USART1_printfMessage(" uT\n\r");
				USART1_printfMessage(" mz = "); USART1_SendFloat(mz); USART1_printfMessage(" uT\n\r");
        
				USART1_printfMessage("\n\rq0 = "); USART1_SendFloat(*getQ());
        USART1_printfMessage("\n\rqx = "); USART1_SendFloat(*(getQ() + 1));
        USART1_printfMessage("\n\rqy = "); USART1_SendFloat(*(getQ() + 2));
        USART1_printfMessage("\n\rqz = "); USART1_SendFloat(*(getQ() + 3));
		}
		
		
		yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() * *(getQ()+3)),
									*getQ() * *getQ() + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3)) * (180 / 3.14159265358979);
		pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()	* *(getQ()+2))) * (180 / 3.14159265358979);
		roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)	* *(getQ()+3)), 
									*getQ() * *getQ() - *(getQ()+1)	* *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)	* *(getQ()+3)) * (180 / 3.14159265358979);
		/*
		yaw   = atan2(2.0f * (SEq_2 * SEq_3 + SEq_1 * SEq_4),
									SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4) * (180 / 3.14159265358979);
		pitch = -asin(2.0f * (SEq_2 * SEq_4 - SEq_1	* SEq_3)) * (180 / 3.14159265358979);
		roll  = atan2(2.0f * (SEq_1 * SEq_2 + SEq_3	* SEq_4), 
									SEq_1 * SEq_1 - SEq_2 * SEq_2 - SEq_3 * SEq_3 + SEq_4 * SEq_4) * (180 / 3.14159265358979);
		*/
		//gimbal lock rijesenje
		//http://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
		if(pitch==90)  {roll=0; yaw=-2*atan2((*getQ()+1),(*getQ()));}
		if(pitch==-90) {roll=0; yaw=2*atan2((*getQ()+1),(*getQ()));}
		//Declination: Croatia, Zagreb, 2021-01-14	4.45° E  ± 0.36°  changing by  0.14° E per year
		yaw -= (float)4.45;
		sprintf(str, "\r%f\t%f\t%f", yaw, pitch, roll);
		USART1_printfMessage(str);

	
	}//while(1)	;
	
}
	
