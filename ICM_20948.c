#include "stm32f4xx_i2c.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_delay.h"
#include "ICM_20948.h"
#include "math.h"
#include "AHRS.h"


//jos nisam iskoristio
float factoryMagCalibration[3] = {0,0,0}, factoryMagBias[3] = {0,0,0}; // Factory mag calibration and mag bias 
//do tud

static uint32_t ID_I2C_Timeout;

// Bias corrections for gyro, accelerometer, and magnetometer
float magBias[3] = {0,0,0},
			magScale[3];
float selfTest[6];
char 	str[50];

AccConfig_t AccConfig;
GyroConfig_t GyroConfig;
uint8_t magmode = MAG_10Hz; //100 mjerenja svih osi skupa u sekundi, dakle 300 uzoraka

void AccGyroParam(GyroConfig_t *GyroConfig, AccConfig_t *AccConfig) {
  AccConfig->accel_DLPF = DLPF_ACC_24HZ;
  AccConfig->accel_range = ACCEL_RANGE_2G;
  AccConfig->accel_averaging = ACCEL_AVG_1X;
  AccConfig->accel_DLPFenable = 1;
  AccConfig->accel_smplrate = ACCEL_SMPLRT_1kHz;


  GyroConfig->gyro_DLPF = DLPF_GYRO_12HZ;
  GyroConfig->gyro_range = GYRO_RANGE_250DPS;
  GyroConfig->gyro_averaging = GYRO_AVG_128X;
  GyroConfig->gyro_DLPFenable = 1;
  GyroConfig->gyro_smplrate = GYRO_SMPLRT_1kHz;
}

float Accel_Resolution;
float Gyro_Resolution;
void USART1_printfMessage(char * nes);

void ID_SelectUserBank(I2C_TypeDef * I2Cx, ICM_UserBank_t UserBank) {
  if (UserBank == ICM_UserBank_0)
    TM_I2C_Write(I2Cx, ICM_20948_BASE, BANK_SEL, REG_BANK_0);
  else if (UserBank == ICM_UserBank_1)
    TM_I2C_Write(I2Cx, ICM_20948_BASE, BANK_SEL, REG_BANK_1);
  else if (UserBank == ICM_UserBank_2)
    TM_I2C_Write(I2Cx, ICM_20948_BASE, BANK_SEL, REG_BANK_2);
  else if (UserBank == ICM_UserBank_3)
    TM_I2C_Write(I2Cx, ICM_20948_BASE, BANK_SEL, REG_BANK_3);
}

void ID_WhoAmI(I2C_TypeDef * I2Cx, uint8_t address) {
  if (ICM_20948_BASE == address && TM_I2C_Read(I2Cx, address, WAI_ICM)) {
    USART1_printfMessage("\tSpojen je ICM20948 na adresi 0x");
    USART1_DecToHexSend(address);
    USART1_printfMessage("\r\n\r");
    return;
  } else if (AK_09916 == address && TM_I2C_Read(I2Cx, address, WAI_AK)) {
    USART1_printfMessage("\tSpojen je AK09916 na adresi 0x");
    USART1_DecToHexSend(address);
    USART1_printfMessage("\r\n\r");
    return;
  } else
    return;
}

void ID_ConfigPowerManagement(I2C_TypeDef * I2Cx, ICM_PwrMgmt_t PwrMgmt) {
  uint8_t data = 0;
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  if (PwrMgmt == ICM_PwrMgmt_1)
    data = DEVICE_RESET | NOSLEEP | LP_DIS | TEMP_DIS | AUTO_SEL; /*vrijednosti mos mijenjat u ICM_20948.h*/
  TM_I2C_Write(I2Cx, ICM_20948_BASE, REG_PWR_MGMT_1, data);
  Delayms(200);
  if (PwrMgmt == ICM_PwrMgmt_2)
    data = ENABLE_GYRO_ACCEL; /*vrijednosti mos mijenjat u ICM_20948.h*/
  TM_I2C_Write(I2Cx, ICM_20948_BASE, REG_PWR_MGMT_2, data);
  Delayms(200);
}

void ID_GyroConfig(I2C_TypeDef * I2Cx, GyroConfig_t *GyroConfig) {
  uint8_t data, regval;

	data = (GyroConfig->gyro_DLPF << 3) | (GyroConfig->gyro_range << 1) | (GyroConfig->gyro_DLPFenable); //GYRO_FCHOICE=1 je low-noise mode
	ID_SelectUserBank(I2Cx, ICM_UserBank_2);	
	TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_1, data);
	regval = TM_I2C_Read(I2Cx, ICM_20948_BASE, GYRO_CONFIG_1);
	sprintf(str, "\n\rupisao sam data %d a procitao regval %d\n\r", data, regval);
	USART1_printfMessage(str);
  TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_2, GyroConfig->gyro_averaging);
	TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_SMPLRT_DIV, GyroConfig->gyro_smplrate); //frekvencija citanja podataka	
	ID_SelectUserBank(I2Cx, ICM_UserBank_0);
}

void ID_AccConfig(I2C_TypeDef * I2Cx, AccConfig_t *AccConfig) {
  uint8_t data;

  data = (AccConfig->accel_DLPF << 3) | (AccConfig->accel_range << 1) | (AccConfig->accel_DLPFenable); //ACCEL_FCHOICE 1 je low-noise mode
	ID_SelectUserBank(I2Cx, ICM_UserBank_2);
  TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_1, data);
  TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_2, AccConfig->accel_averaging);
  TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_SMPLRT_LSB, (uint8_t) AccConfig->accel_smplrate);
  TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_SMPLRT_MSB, (uint8_t)(AccConfig->accel_smplrate >> 8));
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
}

uint8_t ID_MagSelftest(I2C_TypeDef * I2Cx) {
  uint8_t magdata[6], i;
  uint16_t mag_data[3];
  uint8_t neg, correctx = 0, correcty = 0, correctz = 0;
  TM_I2C_Write(I2Cx, AK, CNTL2, MAG_POWER_DOWN_MODE);
  TM_I2C_Write(I2Cx, AK, CNTL2, MAG_SELFTEST_MODE);
  ID_I2C_Timeout = TM_I2C_TIMEOUT;
  while (!(TM_I2C_Read(I2Cx, AK, ST1) & 0x01)) { //wait until data is ready - DRDY flag
    if (--ID_I2C_Timeout == 0x00) {
      USART1_printfMessage("\tmagnetometer timed out during self-test\n\r");
      return (correctx & correcty & correctz);
    }
  }
  TM_I2C_ReadMulti(I2Cx, AK, HXL, magdata, 6);
  TM_I2C_Read(I2Cx, AK, ST2);
  for (i = 0; i < 5; i += 2)
    mag_data[i / 2] = (magdata[(i) + 1] << 8) | magdata[(i)];

  for (i = 0; i < 5; i += 2) {
    neg = 0;
    if ((mag_data[i / 2] & (1 << 15)) != 0) {
      mag_data[i / 2] = (~mag_data[i / 2]) + 1;
      neg = 1;
    }
    switch (i) {
    case 0:
      if (mag_data[i / 2] < 200) {
        USART1_printfMessage("\n\r\tX axis is correct (-200 do 200), value: ");
        correctx = 1;
      }
      if (mag_data[i / 2] > 200) {
        USART1_printfMessage("\n\r\tX axis is incorrect (-200 do 200), value: ");
        correctx = 0;
      }
      if (neg == 1) {
        USART1_printfMessage("-");
      }
      USART1_SendNumber(mag_data[i / 2]);
      USART1_printfMessage("\n\r");
      break;
    case 2:
      if (mag_data[i / 2] < 200) {
        USART1_printfMessage("\n\r\tY axis is correct (-200 do 200), value: ");
        correcty = 1;
      }
      if (mag_data[i / 2] > 200) {
        USART1_printfMessage("\n\r\tYaxis is incorrect (-200 do 200), value: ");
        correcty = 0;
      }
      if (neg == 1) {
        USART1_printfMessage("-");
      }
      USART1_SendNumber(mag_data[i / 2]);
      USART1_printfMessage("\n\r");
      break;
    case 4:
      if (mag_data[i / 2] > 200 && mag_data[i / 2] < 1000 && neg) {
        correctz = 1;
        USART1_printfMessage("\n\r\tZ axis is correct(-200 do -1000), value: -");
        USART1_SendNumber(mag_data[i / 2]);
        USART1_printfMessage("\n\r");
      } else {
        correctz = 0;
        USART1_printfMessage("\n\r\tZ is incorrect(-200 do -1000), value: ");
        if (neg == 1) {
          USART1_printfMessage("-");
        }
        USART1_SendNumber(mag_data[i / 2]);
        //USART1_printfMessage("\n\r");
      }
      break;
    }
  }
	ID_MagnConfig(I2Cx, magmode);
	if(!(correctx & correcty & correctz)){
		sprintf(str,"\n\rprovjeravam valjaju li osi magnetometra\n\r");
		USART1_printfMessage(str);
		}
  return (correctx & correcty & correctz);
}

void ID_MagnConfig(I2C_TypeDef * I2Cx, uint8_t magmode) {
	uint8_t data;
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
	data=TM_I2C_Read(I2Cx, ICM_20948_BASE, INT_PIN_CFG);
	data |= BYPASS_EN;
  TM_I2C_Write(I2Cx, ICM_20948_BASE, INT_PIN_CFG, data);
	Delayms(200);
  TM_I2C_Write(I2Cx, AK, CNTL3, MAG_RESET);
  TM_I2C_Write(I2Cx, AK, CNTL2, magmode);
}

void ID_ICM20948_Init(I2C_TypeDef * I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig) {
  ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_1);
  ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_2);
	ID_SelectUserBank(I2Cx, ICM_UserBank_2);
	TM_I2C_Write(I2Cx, ICM_20948_BASE, ODR_ALIGN_EN, ENABLE);
	ID_SelectUserBank(I2Cx, ICM_UserBank_0);
//	TM_I2C_Write(I2Cx, ICM_20948_BASE, INT_PIN_CFG, 0x20);
//	TM_I2C_Write(I2Cx, ICM_20948_BASE, INT_ENABLE_1, ENABLE);
	AccGyroParam(GyroConfig, AccConfig);
  ID_GyroConfig(I2Cx, GyroConfig);
  ID_AccConfig(I2Cx, AccConfig);
  /*magnetometar treba podesit kao vanjski senzor,
  vidi pass-through mode u datasheetu 24. str
  INT_PIN_CFG, bit 1 uz uvjet da je I/F disablean -passthrough mode za magn.
  */
  ID_MagnConfig(ID_I2C, magmode);
  Delayms(100);
}

void ID_ReadAcc(I2C_TypeDef * I2Cx, int16_t * accel_data) {
  int16_t Xaccel_data, Yaccel_data, Zaccel_data;
  ID_SelectUserBank(I2C3, ICM_UserBank_0);
  Xaccel_data = (int16_t)((TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_XOUT_H)) << 8) |
    (TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_XOUT_L) << 0);
  Yaccel_data = (int16_t)((TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_YOUT_H)) << 8) |
    (TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_YOUT_L) << 0);
  Zaccel_data = (int16_t)((TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_ZOUT_H)) << 8) |
    (TM_I2C_Read(I2C3, ICM_20948_BASE, ACCEL_ZOUT_L) << 0);
  *(accel_data + 0) = Xaccel_data;
  *(accel_data + 1) = Yaccel_data;
  *(accel_data + 2) = Zaccel_data;
  return;
}

void ID_ReadGyro(I2C_TypeDef * I2Cx, int16_t * gyro_data) {
  uint16_t Xgyro_data, Ygyro_data, Zgyro_data;
  ID_SelectUserBank(I2C3, ICM_UserBank_0);
  Xgyro_data = (int16_t)(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_XOUT_H) << 8) |
												(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_XOUT_L) << 0);
  Ygyro_data = (int16_t)(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_YOUT_H) << 8) |
												(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_YOUT_L) << 0);
  Zgyro_data = (int16_t)(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_ZOUT_H) << 8) |
												(TM_I2C_Read(I2C3, ICM_20948_BASE, GYRO_ZOUT_L) << 0);
  *(gyro_data + 0) = Xgyro_data;
  *(gyro_data + 1) = Ygyro_data;
  *(gyro_data + 2) = Zgyro_data;
  return;
}

uint8_t ID_MagDataReady(I2C_TypeDef *I2Cx){
	return (TM_I2C_Read(I2Cx, AK, ST1) & 0x01);
}

void ID_ReadMagCont(I2C_TypeDef * I2Cx, int16_t * mag_data) {
  uint8_t magdata[6], i;
  ID_I2C_Timeout = TM_I2C_TIMEOUT;
  while (!ID_MagDataReady(I2Cx)) { //wait until data is ready - DRDY flag
    if (--ID_I2C_Timeout == 0x00) {
      USART1_printfMessage("magnetometer timed out\n\r");
      return;
    }
  }
  TM_I2C_ReadMulti(I2Cx, AK, HXL, magdata, 6);
  TM_I2C_Read(I2Cx, AK, ST2);
  for (i = 0; i < 5; i += 2) {
    *(mag_data + i/2) = (magdata[(i) + 1] << 8) | magdata[(i)];
  }
  return;
}

void ID_StopMag(I2C_TypeDef * I2Cx) {
  TM_I2C_Write(I2Cx, AK, CNTL2, MAG_POWER_DOWN_MODE);
  return;
}

void ID_ReadMagSingle(I2C_TypeDef * I2Cx, int16_t * mag_data) {
  uint8_t magdata[6], i;
  ID_I2C_Timeout = TM_I2C_TIMEOUT;
  TM_I2C_Write(I2Cx, AK, CNTL2, MAG_SINGLE_MODE);
  while (!(TM_I2C_Read(I2Cx, AK, ST1) & 0x01)) { //wait until data is ready - DRDY flag
    if (--ID_I2C_Timeout == 0x00) {
      USART1_printfMessage("magnetometer timed out\n\r");
      return;
    }
  }
  TM_I2C_ReadMulti(I2Cx, AK, HXL, magdata, 6);
  TM_I2C_Read(I2Cx, AK, ST2);
  for (i = 0; i < 5; i += 2) {
    mag_data[i / 2] = (magdata[(i) + 1] << 8) | magdata[(i)]; //HXL, HXH,... HZH
  }
  return;
}

void ID_ReadSensorData(I2C_TypeDef * I2Cx, ICM_sensor_t sensor, int16_t * data) {
  ID_SelectUserBank(I2C3, ICM_UserBank_0);
  if (sensor == ICM_Accel) {
    ID_ReadAcc(I2Cx, data);
  }
  if (sensor == ICM_Gyro) {
    ID_ReadGyro(I2Cx, data);
  }
  if (sensor == ICM_Magn) {
    ID_ReadMagCont(I2Cx, data); //readmagsingle ce trosit manje energije
  }
  return;
}

float ID_getAccelRes(void) {
  //sve je duplo jer je fullscale +/-
  switch (AccConfig.accel_range) { //accel_range je definiran na pocetku
  case (ACCEL_RANGE_2G):
    return Accel_Resolution = 4. / 65536;
  case (ACCEL_RANGE_4G):
    return Accel_Resolution = 8. / 65536;
  case (ACCEL_RANGE_8G):
    return Accel_Resolution = 16. / 65536;
  case (ACCEL_RANGE_16G):
    return Accel_Resolution = 32. / 65536;
  default:
    return 4. / 65536;
  }
}
float ID_getGyroRes(void) {
  //sve je duplo jer je fullscale +/-
  switch (GyroConfig.gyro_range) {
  case (GYRO_RANGE_250DPS):
    return Gyro_Resolution = 500. / 65536;
  case (GYRO_RANGE_500DPS):
    return Gyro_Resolution = 1000. / 65536;
  case (GYRO_RANGE_1000DPS):
    return Gyro_Resolution = 2000. / 65536;
  case (GYRO_RANGE_2000DPS):
    return Gyro_Resolution = 4000. / 65536;
  default:
    return 500. / 65536;
  }
}
float ID_getMagnRes(void) {
  float Magn_Resolution = 0.15; //0.15 uT - sensitivity, fiksni podatak iz datasheeta
  return Magn_Resolution;
}

void ID_EnableSelfTest(I2C_TypeDef * I2Cx, uint8_t sensor) {
  ID_SelectUserBank(I2Cx, ICM_UserBank_2);
  if (sensor == ENABLE_GYRO_ACCEL) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_2, ACCEL_ST_ENABLE_ALL_AXES);
    TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_2, GYRO_ST_ENABLE_ALL_AXES);
  } else if (sensor == ENABLE_GYRO) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_2, GYRO_ST_ENABLE_ALL_AXES);
  } else if (sensor == ENABLE_ACCEL) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_2, ACCEL_ST_ENABLE_ALL_AXES);
    Delayms(200); // Delay a while to let the device stabilize
    ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  }
  return;
}

void ID_DisableSelfTest(I2C_TypeDef * I2Cx, uint8_t sensor) {
  ID_SelectUserBank(I2Cx, ICM_UserBank_2);
  if (sensor == ENABLE_GYRO_ACCEL) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_2, ACCEL_ST_DISABLE_ALL_AXES);
    TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_2, GYRO_ST_DISABLE_ALL_AXES);
  } else if (sensor == ENABLE_GYRO) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, GYRO_CONFIG_2, GYRO_ST_DISABLE_ALL_AXES);
  } else if (sensor == ENABLE_ACCEL) {
    TM_I2C_Write(I2Cx, ICM_20948_BASE, ACCEL_CONFIG_2, ACCEL_ST_DISABLE_ALL_AXES);
    Delayms(200); // Delay a while to let the device stabilize
    ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  }
  return;
}

void ID_ICM20948SelfTest(I2C_TypeDef * I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig) {
  float destination[6];
  int16_t rawData[3] = {0,0,0};
  uint8_t i, ii;
  int16_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;
	
	ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_1);
  ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_2);
	
	GyroConfig->gyro_DLPF = DLPF_GYRO_200HZ;
  GyroConfig->gyro_range = GYRO_RANGE_250DPS;
  GyroConfig->gyro_averaging = GYRO_AVG_1X;
  GyroConfig->gyro_DLPFenable = 1;
  GyroConfig->gyro_smplrate = GYRO_SMPLRT_1kHz;
  ID_GyroConfig(I2Cx, GyroConfig);
	
  AccConfig->accel_DLPF = DLPF_ACC_250HZ;
  AccConfig->accel_range = ACCEL_RANGE_2G;
  AccConfig->accel_averaging = ACCEL_AVG_1X;
  AccConfig->accel_DLPFenable = 1;
  AccConfig->accel_smplrate = ACCEL_SMPLRT_1kHz;
  ID_AccConfig(I2Cx, AccConfig);

  // Get average current values of gyro and acclerometer
  for (ii = 0; ii < 200; ii++) {
    ID_ReadSensorData(ID_I2C, ICM_Accel, rawData);
    aAvg[0] += rawData[0]; //x accel
    aAvg[1] += rawData[1]; //y accel
    aAvg[2] += rawData[2]; //z accel

    ID_ReadSensorData(ID_I2C, ICM_Gyro, rawData);
    gAvg[0] += rawData[0]; //x Gyro
    gAvg[1] += rawData[1]; //y Gyro
    gAvg[2] += rawData[2]; //z Gyro
  }
	
  // Get average of 200 values and store as average current readings
  for (ii = 0; ii < 3; ii++) {
    aAvg[ii] /= roundf(200.f);
    gAvg[ii] /= 200;
  }

  ID_EnableSelfTest(ID_I2C, ENABLE_GYRO_ACCEL); //self-test data is stored in regular output registers
  // Get average self-test values of gyro and acclerometer
  for (ii = 0; ii < 200; ii++) {
    ID_ReadSensorData(ID_I2C, ICM_Accel, rawData);
    aSTAvg[0] += rawData[0]; //x accel
    aSTAvg[1] += rawData[1]; //y accel
    aSTAvg[2] += rawData[2]; //z accel

    ID_ReadSensorData(ID_I2C, ICM_Gyro, rawData);
    gSTAvg[0] += rawData[0]; //x Gyro
    gSTAvg[1] += rawData[1]; //y Gyro
    gSTAvg[2] += rawData[2]; //z Gyro
  }

  // Get average of 200 values and store as average self-test readings
  for (ii = 0; ii < 3; ii++) {
    aSTAvg[ii] /= 200.f;
    gSTAvg[ii] /= 200.f;
  }

  ID_DisableSelfTest(ID_I2C, ENABLE_GYRO_ACCEL);

  // Retrieve accelerometer and gyro factory Self-Test Code
  ID_SelectUserBank(I2Cx, ICM_UserBank_1);
  selfTest[0] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_X_ACCEL);
  selfTest[1] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_Y_ACCEL);
  selfTest[2] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_Z_ACCEL);
  selfTest[3] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_X_GYRO);
  selfTest[4] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_Y_GYRO);
  selfTest[5] = TM_I2C_Read(I2C3, ICM_20948_BASE, SELF_TEST_Z_GYRO);
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);

  // Retrieve factory self-test value FROM self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[0] - 1.0f)));
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[1] - 1.0f)));
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[2] - 1.0f)));
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[3] - 1.0f)));
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[4] - 1.0f)));
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01f, ((float) selfTest[5] - 1.0f)));
	
	sprintf(str, "\n\rfactory trim %f, selftest %f, FS %d, aSTAvg %d, aAvg %d\n\r", factoryTrim[0], selfTest[0], FS,
	aSTAvg[i], aAvg[i]);
	USART1_printfMessage(str);

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (i = 0; i < 3; i++) {
    // Report percent differences
    destination[i] = (float)(100.0f * ((float)(aSTAvg[i] - aAvg[i])) / (float) factoryTrim[i]-1);
    // Report percent differences
    destination[i + 3] = (float)(100.0f * ((float)(gSTAvg[i] - gAvg[i])) / (float) factoryTrim[i + 3]-1);
  }
  //ovo ti mozda nije dobro jer je destination[] uint8 a sendnumber je int16
	sprintf(str, "\n\rx-axis self test: acceleration trim within : %f %% of factory value", destination[0]);
	USART1_printfMessage(str);
	sprintf(str, "\n\ry-axis self test: acceleration trim within : %f %% of factory value", destination[1]);
	USART1_printfMessage(str);
	sprintf(str, "\n\rz-axis self test: acceleration trim within : %f %% of factory value", destination[2]);
	USART1_printfMessage(str);
	sprintf(str, "\n\rx-axis self test: gyro trim within : %f %% of factory value", destination[3]);
	USART1_printfMessage(str);
	sprintf(str, "\n\ry-axis self test: gyro trim within : %f %% of factory value", destination[4]);
	USART1_printfMessage(str);
	sprintf(str, "\n\rz-axis self test: gyro trim within : %f %% of factory value", destination[5]);
	USART1_printfMessage(str);

  return;
}

void ID_FIFOReset(I2C_TypeDef * I2Cx) {
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, FIFO_RST, FIFO_ASSERT);
  Delayms(10);
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, FIFO_RST, FIFO_DEASSERT);
  Delayms(15);
}
void ID_DisableAllInterrupts(I2C_TypeDef * I2Cx) {
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, INT_ENABLE, DISABLE_ALL); //disable all interupts
  return;
}

void ID_ConfigFIFO(I2C_TypeDef * I2Cx, uint8_t fifo_en_dis, uint8_t who_writes_to_fifo, uint8_t fifo_mode) {
  uint8_t data=0;
  ID_SelectUserBank(I2Cx, ICM_UserBank_0);
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, FIFO_EN_1, DISABLE_ALL); // Enable/Disable slaves writing to FIFO
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, FIFO_EN_2, who_writes_to_fifo); // Enable/Disable internal sensors writing to FIFO
	TM_I2C_Read(ID_I2C, USER_CTRL, data);
  if (fifo_en_dis == FIFO_EN) data |= FIFO_EN; //postavit ce FIFO_EN bit u 1, ostalo ce ostavit kako je
  if (fifo_en_dis == FIFO_DIS) data &= !FIFO_EN; //postavit ce FIFO_EN bit u 0, ostalo ce ostavit kako je
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, USER_CTRL, data); // Enable/Disable FIFO access from serial interface
  TM_I2C_Write(ID_I2C, ICM_20948_BASE, FIFO_MODE, fifo_mode); //snapshot or stream fifo mode
}

void ID_ICM20948Calibration(I2C_TypeDef * I2Cx, GyroConfig_t *GyroConfig, AccConfig_t *AccConfig, float *x, float *y, float *z, int32_t *gyro_bias, int32_t *accel_bias) {
  uint16_t bibocount;
	float		gyroBias[3] = {0,0,0},
					accelBias[3] = {0,0,0};
  uint16_t gyrosensitivity;
  uint16_t accelsensitivity;
	int16_t gyrobibodata[3], accbibodata[3];
	uint32_t ID_I2C_Timeout = 5000;

  ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_1); // reset sensor and Auto select clock source to be PLL
	ID_DisableAllInterrupts(I2Cx); //Disable all interupts


  ID_ConfigPowerManagement(I2Cx, ICM_PwrMgmt_2);
  //ovo samo unutar Calibration funkcije mijenja postavke akcelerometra i gyroskopa
  GyroConfig->gyro_DLPF = DLPF_GYRO_200HZ; 
  GyroConfig->gyro_range = GYRO_RANGE_250DPS;
  GyroConfig->gyro_averaging = GYRO_AVG_1X;
  GyroConfig->gyro_DLPFenable = 1;
  GyroConfig->gyro_smplrate = GYRO_SMPLRT_1kHz; //1kHz
  ID_GyroConfig(I2Cx, GyroConfig);

  AccConfig->accel_DLPF = DLPF_ACC_250HZ;
  AccConfig->accel_range = ACCEL_RANGE_2G;
  AccConfig->accel_averaging = ACCEL_AVG_1X;
  AccConfig->accel_DLPFenable = 1;
  AccConfig->accel_smplrate = ACCEL_SMPLRT_1kHz;
  ID_AccConfig(I2Cx, AccConfig);
	Delayms(200);    // The accel sensor needs max 30ms, the gyro max 35ms to fully start 
	// Experiments show that the gyro needs more time to get reliable results 
  //na githubu je AccConfig.accel_DLPFenable tj. FCHOICE=0 pa je bandwith 1.2 kHZ jer je bypassan DLPF

  accelsensitivity = 32767; //(1/(2*2/65535)) ovo *2 jer je fullscale +/-
  gyrosensitivity = 262; //(1/(250*2/65535)) 

	//dakle imam 16384 bita za prikazat 1G, ako ih moram prikazat +/-2 (tj. 4)
	//2(senzora)*3(osi)*2(bajta, velicina svake osi)*40ms=12*40ms=480 bytea uzoraka na FIFOu
	bibocount = 0;
	while ( bibocount < 500 ) {
		ID_ReadSensorData(ID_I2C, ICM_Gyro, gyrobibodata);
		ID_ReadSensorData(ID_I2C, ICM_Accel, accbibodata);
		if(!(*(x+bibocount-1)==gyrobibodata[0] || *(y+bibocount-1)==gyrobibodata[1] || *(z+bibocount-1)==gyrobibodata[2] ||
			*(x+bibocount+500-1)==accbibodata[0] || *(y+bibocount+500-1)==accbibodata[1] || *(z+bibocount+500-1)==accbibodata[2])){

		
			*(x+bibocount)=(float)gyrobibodata[0];
			*(y+bibocount)=(float)gyrobibodata[1];
			*(z+bibocount)=(float)gyrobibodata[2];
			*(x+bibocount+500)=(float)accbibodata[0];
			*(y+bibocount+500)=(float)accbibodata[1];
			*(z+bibocount+500)=(float)accbibodata[2];

			*(gyro_bias+0) += gyrobibodata[0];
			*(gyro_bias+1) += gyrobibodata[1];
			*(gyro_bias+2) += gyrobibodata[2];
			*(accel_bias+0) += accbibodata[0];
			*(accel_bias+1) += accbibodata[1];
			*(accel_bias+2) += accbibodata[2];
			bibocount++;
			ID_I2C_Timeout = 5000; //ako je uspio procitat podatak, resetiraj timer
		}
			if (--ID_I2C_Timeout == 0x00) {
					sprintf(str,"\n\rICM calibration timeout\n\rNEUSPJESNA kalibracija Akcelerometra i Ziroskopa");
					USART1_printfMessage(str);
					return;
				}
	}
	bibocount=500;
  //1 paket podataka su sve 3 osi oba senzora svake milisekunde, dakle 480(bytea)/6(osi)=40 paketa bi trebalo bit
  //izracunaj srednju vrijednost svih mjerenja
  *(accel_bias+0) =  (int32_t)((float)*(accel_bias+0)/(float)bibocount);
  *(accel_bias+1) =  (int32_t)((float)*(accel_bias+1)/(float)bibocount);
  *(accel_bias+2) =  (int32_t)((float)*(accel_bias+2)/(float)bibocount);
  *(gyro_bias+0) 	=  (int32_t)((float)*(gyro_bias+0)/(float)bibocount);
  *(gyro_bias+1) 	=  (int32_t)((float)*(gyro_bias+1)/(float)bibocount);
  *(gyro_bias+2) 	=  (int32_t)((float)*(gyro_bias+2)/(float)bibocount);
	
  // Output scaled gyro biases for printf
  gyroBias[0] = (float) *(gyro_bias+0);// / (float) gyrosensitivity;
  gyroBias[1] = (float) *(gyro_bias+1);// / (float) gyrosensitivity;
  gyroBias[2] = (float) *(gyro_bias+2);// / (float) gyrosensitivity;
	sprintf(str,"\n\rgyrobias which is being corrected %f %f %f\n\r", gyroBias[0], gyroBias[1], gyroBias[2]);
	USART1_printfMessage(str);

  //ako je Z os akcelerometra negativna, onda mu dodamo 1G (16384 bita) kako bi ga stavili na nulu
  //odnosno oduzmemo mu 1G da ga stavimo na nulu
  if (*(accel_bias+2) > 0L) {
    *(accel_bias+2) -= (int32_t) accelsensitivity/2; // podijeljeno s 2 jer je +2G osjetljivost a moramo uduzet 1G
  } else {
    *(accel_bias+2) += (int32_t) accelsensitivity/2;
  }

  // Output scaled accelerometer biases for printf
  accelBias[0] = (float) *(accel_bias+0);//*ID_getAccelRes(); /// (float) accelsensitivity;
  accelBias[1] = (float) *(accel_bias+1);//*ID_getAccelRes(); /// (float) accelsensitivity;
  accelBias[2] = (float) *(accel_bias+2);//*ID_getAccelRes(); /// (float) accelsensitivity;
	sprintf(str,"accelBias which is being corrected %f %f %f\n\r", accelBias[0], accelBias[1], accelBias[2]);
	USART1_printfMessage(str);

	ID_ICM20948_Init(ID_I2C, GyroConfig, AccConfig);
}

uint8_t ID_MagnCalibration(I2C_TypeDef * I2Cx, float *x, float *y, float *z, float *kappa, float *sigma, uint16_t *sample_count, float  *qkoso, float *qzos, float *mag_bias) {

  //fi oko x, theta oko y, psi oko z


	uint8_t Tsamp;
  float mag_max[3] = {-65000,-65000,-65000},
				mag_min[3] = {65000,65000,65000};
	int16_t tag, mag_temp[3];
//  int16_t mag_temp[3] = {0,0,0};
  float thex = 0, they = 0, thez = 0;
  uint16_t ii = 0;
  float p[4], qconj[4], prot[4], rez[4], zos[3] = {0,0,1}, a[4];
  float qnorm;
  float alfa, betha,
				xy, xyz,
				xyzmax = 0, xymax = 0,
				fi, theta, psi,
				salfah, calfah,
				avg = 0;
  float dulja, kraca;
//  uint32_t Tsamp; //in milisec
//  float MagnRes, avg_rad;
	char c;
	uint8_t debug = 0,
					ispisisve = 0,
					matrica = 0; 

  USART1_printfMessage("\n\rMagnetometer calibration: spin sensor arround itself!");
  USART1_printfMessage("\n\rSampling for 15 seconds");
	USART1_printfMessage("\n\rpress y when ready\n\r");
	while(!(c=='y'))(USART1_Dequeue(&c));
	c=0;
	
  if (magmode == MAG_10Hz) {
    *sample_count = 150; // sampling time/sampling period MORA BIT 150!!!!!! 15 je test
    Tsamp = 100;	
		USART1_printfMessage("MAG_10Hz");
  }
  if (magmode == MAG_20Hz) {
    *sample_count = 300; // sampling time/sampling period
    Tsamp = 50;	
		USART1_printfMessage("MAG_20Hz");
  }
  if (magmode == MAG_50Hz) {
    *sample_count = 750; // sampling time/sampling period
    Tsamp = 20;	
		USART1_printfMessage("MAG_50Hz");
  }
  if (magmode == MAG_100Hz) {
    *sample_count = 1500; // sampling time/sampling period
    Tsamp = 10;	
		USART1_printfMessage("MAG_100Hz");
  }
	
  USART1_printfMessage("\n\rStart spinning\n\r");
  Delayms(300);

  for (ii = 0; ii < *sample_count; ii++) {
		ID_ReadMagCont(I2Cx, mag_temp); // Read the mag data, x,y pa z		
		*(x+ii)=mag_temp[0];
		*(y+ii)=mag_temp[1];
		*(z+ii)=mag_temp[2];

    if (*(x+ii) > mag_max[0]) mag_max[0] = *(x+ii);
    if (*(y+ii) > mag_max[1]) mag_max[1] = *(y+ii);
    if (*(z+ii) > mag_max[2]) mag_max[2] = *(z+ii);

    if (*(x+ii) < mag_min[0]) mag_min[0] = *(x+ii);
    if (*(y+ii) < mag_min[1]) mag_min[1] = *(y+ii);
    if (*(z+ii) < mag_min[2]) mag_min[2] = *(z+ii);
		Delayms(Tsamp*0.05); //10% duze za svaki slucaj. cekamo novi podatak na magnetometru
		
		sprintf(str, "%d. data: %f %f %f\t%f percent\n\r", ii, mag_temp[0]*0.15, mag_temp[1]*0.15, mag_temp[2]*0.15,
						(1.f-(float)(*sample_count-ii)/(float)*sample_count)*100.f);		
		USART1_printfMessage(str);
  }
	if(ispisisve){
		for(ii=0;ii<150;ii++){
			sprintf(str,"[%f, %f, %f], ",*(x+ii),*(y+ii),*(z+ii));
			USART1_printfMessage(str);
		}
				USART1_printfMessage("\n\rsirovi. y za dalje\n\r");
		while(!(c=='y' || c=='Y' || c=='z' || c=='Z'))(USART1_Dequeue(&c));
		c=0;
	}

  *(mag_bias+0) = (mag_max[0] + mag_min[0]) / 2;
  *(mag_bias+1) = (mag_max[1] + mag_min[1]) / 2;
  *(mag_bias+2) = (mag_max[2] + mag_min[2]) / 2;

  /*
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;
  avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2])/3.0;
  magScale[0] = avg_rad / ((float)mag_scale[0]);
  magScale[1] = avg_rad / ((float)mag_scale[1]);
  magScale[2] = avg_rad / ((float)mag_scale[2]);
  */

  for (ii = 0; ii < 3; ii++) {
    sprintf(str, "\n\rmag max %f, mag min %f mag_bias %f", mag_max[ii], mag_min[ii], mag_bias[ii]);
    USART1_printfMessage(str);
  }

  sprintf(str, "\n\rcentrirana\n\r");
  USART1_printfMessage(str);
  for (ii = 0; ii < *sample_count; ii++) {
    *(x+ii) -= *(mag_bias+0);
    *(y+ii) -= *(mag_bias+1);
    *(z+ii) -= *(mag_bias+2);
    xyz = *(x+ii) * *(x+ii) + *(y+ii) * *(y+ii) + *(z+ii) * *(z+ii);

    if (xyzmax < xyz) {
      xyzmax = xyz;
      thex = *(x+ii);
      they = *(y+ii);
      thez = *(z+ii);
      tag = ii;
    }
    if(matrica) {
      sprintf(str, "[%d, %d, %d], ", (int) roundf(*(x+ii)), (int) roundf(*(y+ii)), (int) roundf(*(z+ii)));
      USART1_printfMessage(str);
    }
  }
  for (ii = 0; ii < *sample_count; ii++)
    if (ispisisve) {
      sprintf(str, "%d. uzorak %f %f %f\n\r", ii, *(x+ii), *(y+ii), *(z+ii));
      USART1_printfMessage(str);
    }

  USART1_printfMessage("----------------OPIS POCETNE ELIPSE----------------\n\r");

  xyz = sqrt(thex * thex + they * they + thez * thez);
  if (thex > 0)
    xy = sqrt(thex * thex + they * they);
  if (thex < 0)
    xy = -sqrt(thex * thex + they * they);
  alfa = atan2(xy, thez);

  sprintf(str, "tag %d xyz %f xy %f\n\r thex %f they %f thez %f\n\r", tag, xyz, xy, thex, they, thez);
  USART1_printfMessage(str);
  sprintf(str, "alfa %f\n\r", alfa * (180.f / (pi)));
  USART1_printfMessage(str);

  fi = atan2(thez, they) * (180 / 3.14159265358979);
  theta = atan2(thex, thez) * (180 / 3.14159265358979);
  psi = atan2(they, thex) * (180 / 3.14159265358979);
  sprintf(str, "2tan2gensni2 kutovi: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
  USART1_printfMessage(str);

  //ovo sve treba u RADIANE

  sprintf(str, "\nalfa %f\n\r", alfa * (180.f / pi));
  USART1_printfMessage(str);
  calfah = cos(alfa / 2);
  salfah = sin(alfa / 2);

  //nije normiran -> /xyz ako zelis da bude
  //ni ne smije bit normiran
  //normiranjem P-a mozda mogu izbaciti sigmu i kapu jer je sve na
  //jedinicnoj duljini onda, pa samo * jakost polja sto je min(xyz)
  //	p[0] = sqrt(thex*thex+they*they+thez*thez); //P Length
  p[1] = (float) thex; //x 
  p[2] = (float) they; //y
  p[3] = (float) thez; //z
  p[0] = 0;

  //crossproduct 2 vektora, od kojeg (p) do kojeg vrtim (z)
  a[1] = p[2] * zos[2] - p[3] * zos[1];
  a[2] = -(p[1] * zos[2] - p[3] * zos[0]);
  a[3] = p[1] * zos[1] - p[2] * zos[0];
  sprintf(str, "alfah %f\n\r", alfa * (180.f / (2 * pi)));
  USART1_printfMessage(str);
  sprintf(str, "salfah %f\n\r", salfah);
  USART1_printfMessage(str);
  sprintf(str, "calfah %f\n\r", calfah);
  USART1_printfMessage(str);

  sprintf(str, "\nvrtim od p[1] %f, p[2] %f, p[3] %f\n\r", p[1], p[2], p[3]);
  USART1_printfMessage(str);
  sprintf(str, "vrtim do zos[1] %f, zos[2] %f, zos[3] %f\n\r", zos[0], zos[1], zos[2]);
  USART1_printfMessage(str);
  sprintf(str, "oko a[1] %f, a[2] %f, a[3] %f\n\r", a[1], a[2], a[3]);
  USART1_printfMessage(str);

//  a[0] = sqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]); // Length of a
//  a[1] /= a[0];
//  a[2] /= a[0];
//  a[3] /= a[0];

  a[0] = Q_rsqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]); // Length of a
  a[1] *= a[0];
  a[2] *= a[0];
  a[3] *= a[0];

  *(qkoso+0) = calfah;
  *(qkoso+1) = salfah * a[1]; //x
  *(qkoso+2) = salfah * a[2]; //y
  *(qkoso+3) = salfah * a[3]; //z



  qnorm = Q_rsqrt(*(qkoso+0)**(qkoso+0) + *(qkoso+1)**(qkoso+1) + *(qkoso+2)**(qkoso+2) + *(qkoso+3)**(qkoso+3));
  *(qkoso+0)*=qnorm;
  *(qkoso+1)*=qnorm;
  *(qkoso+2)*=qnorm;
  *(qkoso+3)*=qnorm;
	
//	qnorm = sqrt(*(qkoso+0)**(qkoso+0) + *(qkoso+1)**(qkoso+1) + *(qkoso+2)**(qkoso+2) + *(qkoso+3)**(qkoso+3));
//  *(qkoso+0)/=qnorm;
//  *(qkoso+1)/=qnorm;
//  *(qkoso+2)/=qnorm;
//  *(qkoso+3)/=qnorm;


  quat_conj(qkoso, qconj);
  quat_mult(qkoso, p, prot);
  quat_mult(prot, qconj, rez);

  rez[0] = roundf(rez[0]);
  rez[1] = roundf(rez[1]);
  rez[2] = roundf(rez[2]);
  rez[3] = roundf(rez[3]);

  sprintf(str, "\np je:\n\r %f, %f, %f, %f\n\r", p[0], p[1], p[2], p[3]);
  USART1_printfMessage(str);
  quat_len(p);
  sprintf(str, "qkoso je:\n\r %f, %f, %f, %f\n\r", *(qkoso+0), *(qkoso+1), *(qkoso+2), *(qkoso+3));
  USART1_printfMessage(str);
  sprintf(str, "qkoso* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
  USART1_printfMessage(str);
  sprintf(str, "qkoso*p je:\n\r %f, %f, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
  USART1_printfMessage(str);
  sprintf(str, "uspravljeni p je:\n\r %f, %f, %f, %f\n\r", rez[0], rez[1], rez[2], rez[3]);
  USART1_printfMessage(str);
  quat_len(rez);

  thez = rez[3]; //visina elipse, trebat ce kasnije za skaliranje


  sprintf(str, "Alfa kut rotacije:\n\r %f\n\r", alfa * (180.f / pi));
  USART1_printfMessage(str);
  fi = atan2(rez[3], rez[2]) * (180 / 3.14159265358979);
  theta = atan2(rez[1], rez[3]) * (180 / 3.14159265358979);
  if (rez[2] == rez[1] && rez[2] == 0)
    psi = 0;
  else
    psi = atan2(rez[2], rez[1]) * (180 / 3.14159265358979);
  sprintf(str, "2tan2gensni2 kutovi nakon uspravljanja: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
  USART1_printfMessage(str);

  sprintf(str, "\n-------------USPRAVLJAM ELIPSU----------\n\r");
  USART1_printfMessage(str);
  for (ii = 0; ii < *sample_count; ii++) {
    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);
	
    if (debug == 1 && ii == 10) {
      fi = atan2(p[3], p[2]) * (180 / 3.14159265358979);
      theta = atan2(p[1], p[3]) * (180 / 3.14159265358979);
      psi = atan2(p[2], p[1]) * (180 / 3.14159265358979);
      sprintf(str, "2tan2gensni2 kutovi: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
      USART1_printfMessage(str);
    }

    quat_conj(qkoso, qconj);
    quat_mult(qkoso, p, prot);
    quat_mult(prot, qconj, rez);

    if (debug == 1 && ii == 10) {
      sprintf(str, "%d. tocka p je:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      sprintf(str, "duljina vektora p je %f", quat_len(p));
      sprintf(str, "qkoso je:\n\r %f, %f, %f, %f\n\r", *(qkoso+0), *(qkoso+1), *(qkoso+2), *(qkoso+3));
      USART1_printfMessage(str);
      sprintf(str, "qkoso* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
      USART1_printfMessage(str);
      sprintf(str, "qkoso*p je:\n\r %f, %lf, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
      USART1_printfMessage(str);
    }

    p[0] = roundf(rez[0]);
    p[1] = roundf(rez[1]);
    p[2] = roundf(rez[2]);
    p[3] = roundf(rez[3]);

    //bez roundova malo felšaju kutevi
    //prica o kvantizacijskom šumu od 0.5 LSB

    if (debug == 1 && ii == 10) {
      sprintf(str, "%d. uspravljena tocka p je:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      fi = atan2(rez[3], rez[2]) * (180 / 3.14159265358979);
      theta = atan2(rez[1], rez[3]) * (180 / 3.14159265358979);
      if (rez[2] == rez[1] && rez[2] == 0)
        psi = 0;
      else
        psi = atan2(rez[2], rez[1]) * (180 / 3.14159265358979);
      sprintf(str, "2tan2gensni2 kutovi nakon uspravljanja: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
      USART1_printfMessage(str);
    }

    xy = rez[1] * rez[1] + rez[2] * rez[2]; //x*x+y*y
    if (xy > xymax) {
      xymax = xy;
      thex =  roundf(rez[1]);
      they =  roundf(rez[2]);
    }

    *(x+ii) = roundf(rez[1]);
    *(y+ii) = roundf(rez[2]);
    *(z+ii) = roundf(rez[3]);

  }
  dulja = roundf(sqrt(xymax));
  if (ispisisve) {
    for (ii = 0; ii < *sample_count; ii++){
      sprintf(str, "%d. uzorak %f %f %f\n\r", ii, *(x+ii), *(y+ii), *(z+ii));
			USART1_printfMessage(str);
		}
  }
  if (debug) {
    sprintf(str, "uspravljeni 10.\n\rx=%f y=%f z=%f\n\r", *(x+10), *(y+10), *(z+10));
    USART1_printfMessage(str);
  }
  sprintf(str, "thex %f, they %f, xymax %f\n\r", thex, they, xymax);
  USART1_printfMessage(str);
	if (matrica) {
  sprintf(str, "\n\n\n\n\n\n\n\n\n---------HOL UP----------\n\n\n\r");
	USART1_printfMessage(str);
    for (ii = 0; ii < *sample_count; ii++) {
      sprintf(str, "[%d, %d, %d], ", (int) roundf(*(x+ii)), (int) roundf(*(y+ii)), (int) roundf(*(z+ii)));
      USART1_printfMessage(str);
    }
  sprintf(str, "\n\n\n---------HOL UP----------\n\n\n\n\n\n\n\n\n\r");
  USART1_printfMessage(str);
	}
  sprintf(str, "--------------ROTACIJA OKO Z OSI---------------\n\r");
  USART1_printfMessage(str);
  dulja = roundf(sqrt(xymax)); //dulja duzina do ruba elipse na xy plohi
  if (debug) {
    sprintf(str, "uspravljeni rotirani i SKALIRANI 10.\n\rx=%f y=%f z=%f\n\r", *(x+10), *(y+10), *(z+10));
    USART1_printfMessage(str);
  }
  //rotacija oko z za kut -betha jer trebamo clockwise
  //da bi siri dio bio na x osi, a uzi na y osi
  betha = -atan2(they, thex);
  sprintf(str, "betha oko z je:\n\r %f\n\r", betha * (180 / pi));
  USART1_printfMessage(str);

  *(qzos+0) = cos(betha / 2);
  *(qzos+1) = 0;
  *(qzos+2) = 0;
  *(qzos+3) = sin(betha / 2);

//  qnorm = sqrt(*(qzos+0) * *(qzos+0) + *(qzos+1) * *(qzos+1) + *(qzos+2) * *(qzos+2) + *(qzos+3) * *(qzos+3));
//  *(qzos+0) /= qnorm;
//  *(qzos+1) /= qnorm;
//  *(qzos+2) /= qnorm;
//  *(qzos+3) /= qnorm;
	
	qnorm = Q_rsqrt(*(qzos+0) * *(qzos+0) + *(qzos+1) * *(qzos+1) + *(qzos+2) * *(qzos+2) + *(qzos+3) * *(qzos+3));
  *(qzos+0) *= qnorm;
  *(qzos+1) *= qnorm;
  *(qzos+2) *= qnorm;
  *(qzos+3) *= qnorm;
	

  for (ii = 0; ii < *sample_count; ii++) {
    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);

    quat_conj(qzos, qconj);
    quat_mult(qzos, p, prot);
    quat_mult(prot, qconj, rez);

    if (debug == 1 && ii == 10) {
      sprintf(str, "rotiram %d. tocku p:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      sprintf(str, "qzos je:\n\r %f, %f, %f, %f\n\r", *(qzos+0), *(qzos+1), *(qzos+2), *(qzos+3));
      USART1_printfMessage(str);
      sprintf(str, "qzos* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
      USART1_printfMessage(str);
      sprintf(str, "qzos*p je:\n\r %f, %f, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
      USART1_printfMessage(str);
    }

    p[0] = roundf(rez[0]);
    p[1] = roundf(rez[1]);
    p[2] = roundf(rez[2]);
    p[3] = roundf(rez[3]);

    *(x+ii) = p[1];
    *(y+ii) = p[2];
    *(z+ii) = p[3];

    if (debug == 1 && ii == 10) {
      sprintf(str, "%d. uspravljena i rotirana tocka p je:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      fi = atan2(rez[3], rez[2]) * (180 / 3.14159265358979);
      theta = atan2(rez[1], rez[3]) * (180 / 3.14159265358979);
      if (rez[2] == rez[1] && rez[2] == 0)
        psi = 0;
      else
        psi = atan2(rez[2], rez[1]) * (180 / 3.14159265358979);
      sprintf(str, "2tan2gensni2 kutovi nakon rotacije oko z: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
      USART1_printfMessage(str);
    }
    if (ii == 0)
      they = fabsf(*(y+ii));
    if (they < fabsf(*(y+ii)))
      they = *(y+ii);
  }
  sprintf(str,"\n\n-------ROTIRANA OKO Z NESKALIRANA-------\n\r");
	USART1_printfMessage(str);
  if (matrica)
  for (ii = 0; ii < *sample_count; ii++) {
    sprintf(str,"[%d, %d, %d], ", (int)roundf(*(x+ii)), (int)roundf(*(y+ii)), (int)roundf(*(z+ii)));
		USART1_printfMessage(str);
  }
  if (debug) {
    sprintf(str, "Duljina 10.og rotiranog oko Z osi\n\r");
    USART1_printfMessage(str);
    p[0] = 0;
    p[1] = *(x+10);
    p[2] = *(y+10);
    p[3] = *(z+10);
    quat_len(p);
  }

  kraca = they;
  *sigma = kraca / dulja; //xy ploha
  *kappa = kraca / thez; //xz ploha

  sprintf(str,"Skalirana i rotirana elipsa:\n\r");
	USART1_printfMessage(str);
  for (ii = 0; ii < *sample_count; ii++) {
    *(x+ii) = roundf(*(x+ii) * *sigma);
    *(y+ii) = roundf(*(y+ii));
    *(z+ii) = roundf(*(z+ii) * *kappa);
  }
  sprintf(str, "dulja %f, kraca %f, thez %f\n\r", dulja, kraca, thez);
  USART1_printfMessage(str);
  sprintf(str, "Sigma i Kappa\n\r %f %f\n\r", *sigma, *kappa);
  USART1_printfMessage(str);
	sprintf(str, "qzos\n\r %f %f %f %f", qzos[0], qzos[1], qzos[2], qzos[3]);
	USART1_printfMessage(str);
	sprintf(str, "qkoso\n\r %f %f %f %f", qkoso[0], qkoso[1], qkoso[2], qkoso[3]);
	USART1_printfMessage(str);
  if (debug) {
    sprintf(str, "uspravljeni rotirani i SKALIRANI 10.\n\rx=%f y=%f z=%f\n\r", *(x+10), *(y+10), *(z+10));
    USART1_printfMessage(str);
  }
  if (matrica){
    for (ii = 0; ii < *sample_count; ii++) {
      sprintf(str,"[%d, %d, %d], ", (int) roundf(*(x+ii)), (int) roundf(*(y+ii)), (int) roundf(*(z+ii)));
			USART1_printfMessage(str);
    }
  sprintf(str, "\n\n\r------------ROTIRANA I SKALIRANA----------\n\n\r");
	USART1_printfMessage(str);
	}

  if (debug) {
    p[0] = 0;
    p[1] = *(x+10);
    p[2] = *(y+10);
    p[3] = *(z+10);
		sprintf(str, "Duljina skaliranog P \n\r %f", quat_len(p));
    USART1_printfMessage(str);

  }
  sprintf(str, "\n\r-----------------ODROTIRAJ OKO Z--------------------\n\r");
  USART1_printfMessage(str);
  for (ii = 0; ii < *sample_count; ii++) {
    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);

		quat_conj(qzos, qconj);
		quat_mult(qconj, p, prot);
		quat_mult(prot, qzos, rez);

    if (debug == 1 && ii == 10) {
      sprintf(str, "betha je %f\n\r", betha * 180 / pi);
      USART1_printfMessage(str);
      sprintf(str, "ODrotiram tocku p oko z:\n\r %f, %f, %f, %f\n\r", p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      sprintf(str, "qzos je:\n\r %f, %f, %f, %f\n\r", *(qzos+0), *(qzos+1), *(qzos+2), *(qzos+3));
      USART1_printfMessage(str);
      sprintf(str, "qzos* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
      USART1_printfMessage(str);
      sprintf(str, "qkoso*p je:\n\r %f, %f, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
      USART1_printfMessage(str);
    }

    p[0] = roundf(rez[0]);
    p[1] = roundf(rez[1]);
    p[2] = roundf(rez[2]);
    p[3] = roundf(rez[3]);

    if (debug == 1 && ii == 10) {
      sprintf(str, "%d. odrotirana tocka p oko z je:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      fi = atan2(rez[3], rez[2]) * (180 / 3.14159265358979);
      theta = atan2(rez[1], rez[3]) * (180 / 3.14159265358979);
      if (rez[2] == rez[1] && rez[2] == 0)
        psi = 0;
      else
        psi = atan2(rez[2], rez[1]) * (180 / 3.14159265358979);
      sprintf(str, "2tan2gensni2 kutovi nakon ODrotiranja: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
      USART1_printfMessage(str);
    }

    *(x+ii) = p[1];
    *(y+ii) = p[2];
    *(z+ii) = p[3];
  }
  if (debug) {
    sprintf(str, "ODrotirani i SKALIRANI 10.\n\rx=%f y=%f z=%f\n\r", *(x+10), *(y+10), *(z+10));
    USART1_printfMessage(str);
  }

  sprintf(str, "-----------POLEGNI KUGLU NA POCETNI POLOZAJ-------------\n\r");
  USART1_printfMessage(str);
  //mozda mozes napravit qzos*qkoso pa da odjednom rotiras
  //oko z i polegnes kuglu. mozda je obrnuto -> qkoso*qzos

  for (ii = 0; ii < *sample_count; ii++) {

    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);

		
				//polegni elipsu
		quat_conj(qkoso, qconj);
		quat_mult(qconj, p, prot);
		quat_mult(prot, qkoso, rez);	

    if (debug == 1 && ii == 10) {
      sprintf(str, "polijezem tocku p oko z:\n\r %f, %f, %f, %f\n\r", p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      sprintf(str, "qkoso je:\n\r %f, %f, %f, %f\n\r", *(qkoso+0), *(qkoso+1), *(qkoso+2), *(qkoso+3));
      USART1_printfMessage(str);
      sprintf(str, "qkoso* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
      USART1_printfMessage(str);
      sprintf(str, "qkoso*p je:\n\r %f, %f, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
    }

    p[0] = roundf(rez[0]);
    p[1] = roundf(rez[1]);
    p[2] = roundf(rez[2]);
    p[3] = roundf(rez[3]);
		
    if (debug == 1 && ii == 10) {
      sprintf(str, "%d. polegnuta tocka p oko z je:\n\r %f, %f, %f, %f\n\r", ii, p[0], p[1], p[2], p[3]);
      USART1_printfMessage(str);
      quat_len(p);
      fi = atan2(rez[3], rez[2]) * (180 / 3.14159265358979);
      theta = atan2(rez[1], rez[3]) * (180 / 3.14159265358979);
      if (rez[2] == rez[1] && rez[2] == 0)
        psi = 0;
      else
        psi = atan2(rez[2], rez[1]) * (180 / 3.14159265358979);
      sprintf(str, "2tan2gensni2 kutovi nakon polegnuca: \n\rfi: %f\n\rtheta: %f\n\rpsi: %f\n\r", fi, theta, psi);
      USART1_printfMessage(str);
    }
    *(x+ii) = p[1];
    *(y+ii) = p[2];
    *(z+ii) = p[3];
  }
  if (debug) {
    sprintf(str, "polegnuta 10.\n\rx=%f y=%f z=%f\n\r", *(x+10), *(y+10), *(z+10));
    USART1_printfMessage(str);
  }
  if (matrica) {
    for (ii = 0; ii < *sample_count; ii++) {
      sprintf(str,"[%d, %d, %d], ", (int) roundf(*(x+ii)), (int) roundf(*(y+ii)), (int) roundf(*(z+ii)));
			USART1_printfMessage(str);
    }
  }

  sprintf(str, "------------KONACNA PROVJERA------------\n\r");
  USART1_printfMessage(str);
  for (ii = 0; ii < *sample_count; ii++) {

    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);
		
		if(ispisisve){
    sprintf(str, "%d. ", ii);
    USART1_printfMessage(str);
		sprintf(str, "duljina p je %f\n\r", quat_len(p));
		USART1_printfMessage(str);}
    avg += quat_len(p);

  }
  avg /= *sample_count;
  sprintf(str, "avg %f\n\r", avg);
  USART1_printfMessage(str);
  xymax = 0;
  for (ii = 0; ii < *sample_count; ii++) {
    p[0] = 0;
    p[1] = *(x+ii);
    p[2] = *(y+ii);
    p[3] = *(z+ii);
    xy = fabsf(roundf(quat_len(p) - avg));
    if (xy > xymax) xymax = xy;
  }
  sprintf(str, "maxerr %f\n\r", xy);
  USART1_printfMessage(str);
  xymax = (1 - (4912. - xymax) / 4912.) * 100;
  sprintf(str, "error is %f percent\n\r", xymax);
  USART1_printfMessage(str);

	USART1_printfMessage(str);
	
  if (xymax > 5) {
    sprintf(str, "calibration error is too big, calirate again");
    USART1_printfMessage(str);
    return 0;
  } else {
    sprintf(str, "Successfull calibration!");
    USART1_printfMessage(str);
    return 1;
  }
}

void ID_MagnCorrection(int16_t * MagMeasurement, int debug, float kappa, float sigma, float  qkoso[4], float qzos[4], float mag_bias[3]) {
  float p[4], qconj[4], prot[4], rez[4];
  int i;
 // f(&kappa, &sigma, qkoso, qzos, mag_bias);
	debug=0;
  p[0] = 0;
  p[1] = * (MagMeasurement + 0) - *(mag_bias+0);
  p[2] = * (MagMeasurement + 1) - *(mag_bias+1);
  p[3] = * (MagMeasurement + 2) - *(mag_bias+2);


  if (debug) {
    quat_len(p);
    sprintf(str, "\n\rvektor prije vrtnje\n\r");
    USART1_printfMessage(str);
    for (i = 0; i < 4; i++) {
      sprintf(str, " p[%d]=%f\n\r", i, p[i]);
      USART1_printfMessage(str);
    }
  }

  //uspravljam elipsu
  quat_conj(qkoso, qconj);
  quat_mult(qkoso, p, prot);
  quat_mult(prot, qconj, rez);
	
	if(debug){
  sprintf(str, "\n\rp je:\n\r %f, %f, %f, %f\n\r", p[0], p[1], p[2], p[3]);
  USART1_printfMessage(str);
  quat_len(p);
  sprintf(str, "qkoso je:\n\r %f, %f, %f, %f\n\r", *(qkoso+0), *(qkoso+1), *(qkoso+2), *(qkoso+3));
  USART1_printfMessage(str);
  sprintf(str, "qkoso* je:\n\r %f, %f, %f, %f\n\r", qconj[0], qconj[1], qconj[2], qconj[3]);
  USART1_printfMessage(str);
  sprintf(str, "qkoso*p je:\n\r %f, %f, %f, %f\n\r", prot[0], prot[1], prot[2], prot[3]);
  USART1_printfMessage(str);
  sprintf(str, "uspravljeni p je:\n\r %f, %f, %f, %f\n\n\r", rez[0], rez[1], rez[2], rez[3]);
  USART1_printfMessage(str);
  quat_len(rez);
	sprintf(str, "sigma %f, kappa %f\n\rqkoso %f %f %f %f\n\rqzos %f %f %f %f\n\rmag_bias %f %f %f\n\r",
								sigma, kappa,
								qkoso[0], qkoso[1], qkoso[2], qkoso[3],
								qzos[0], qzos[1], qzos[2], qzos[3],
								mag_bias[0], mag_bias[1], mag_bias[2]);
  USART1_printfMessage(str);
	}

  p[0] = rez[0];
  p[1] = rez[1];
  p[2] = rez[2];
  p[3] = rez[3];

	//rotiram elipsu oko z
  quat_conj(qzos, qconj);
  quat_mult(qzos, p, prot);
  quat_mult(prot, qconj, rez);
	
	//skaliram elipsu
  p[0] = rez[0];
  p[1] = rez[1] * sigma;
  p[2] = rez[2];
  p[3] = rez[3] * kappa;
	
	quat_conj(qzos, qconj);
	quat_mult(qzos, p, prot);
	quat_mult(prot, qconj, rez);
	
	//odrotiram elipsu oko z
	quat_conj(qzos, qconj);
	quat_mult(qconj, p, prot);
	quat_mult(prot, qzos, rez);
	
	p[0]=roundf(rez[0]);
	p[1]=roundf(rez[1]);
	p[2]=roundf(rez[2]);
	p[3]=roundf(rez[3]);

	//polegni elipsu
	quat_conj(qkoso, qconj);
	quat_mult(qconj, p, prot);
	quat_mult(prot, qkoso, rez);	

	p[0]=roundf(rez[0]);
	p[1]=roundf(rez[1]);
	p[2]=roundf(rez[2]);
	p[3]=roundf(rez[3]);

  if (debug) {
    sprintf(str, "FINALNI vektor nakon vrtnje\n\r");
    USART1_printfMessage(str);
    for (i = 0; i < 4; i++) {
      sprintf(str, " p[%d]=%f\n\r", i, p[i]);
      USART1_printfMessage(str);
    }
    quat_len(p);
  }

  *(MagMeasurement + 0) = (int16_t) roundf(p[1]);
  *(MagMeasurement + 1) = (int16_t) roundf(p[2]);
  *(MagMeasurement + 2) = (int16_t) roundf(p[3]);
	
	return;
}

void f(float *kappa, float *sigma, float *qkoso, float *qzos, float *mag_bias) {

  *(qkoso+0) = 0.873840;
  *(qkoso+1) = -0.360574;
  *(qkoso+2) = 0.326175;
  *(qkoso+3) = 0.000000;

  *(qzos+0) = 0.687777;
  *(qzos+1) = 0;
  *(qzos+2) = 0;
  *(qzos+3) = -0.725922;

  *sigma = 0.926003;
  *kappa = 0.868355;

  *(mag_bias+0) = 362.5;
  *(mag_bias+1) = -496.5;
  *(mag_bias+2) = 997.511963;
	
	magScale[0]=0;
	magScale[1]=0;
	magScale[2]=0;

}

/*
  void ID_ICM_20948_INT_user_control(uint8_t DMP, uint8_t FIFO){


-DMP se cita iz FIFO
-frekv. rada DMPa minimalno 200Hz, cak i u low power modeu
-CLKSSEL = 1 -->sam bira clock. dok je samo akselerometar onda je ukljucen interni oscilator, žiroskop u bilo kojoj kombinaciji pali PLL
-FIFO 512 B, u njega se mogu spremati podaci sa senzora i moze generirat interuptove (nemoze jer je pin blokiran bužirom)
-iskljuci interruptove
-iskljuci termometar
-8 power modeova
*/
