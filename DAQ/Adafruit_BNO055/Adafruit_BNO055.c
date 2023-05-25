#include "main.h"

#include "stdlib.h"
#include "stdio.h"

#include "Adafruit_BNO055.h"

uint8_t _address;
int32_t _sensorID;
adafruit_bno055_opmode_t _mode;

/*pointer to save sensor I2c configrations prameters*/
static I2C_HandleTypeDef* GlobalConfig = NULL;

void Adafruit_BNO055_Init(I2C_HandleTypeDef* Config, int32_t sensorID, uint8_t address)
{
	_sensorID = sensorID;
	_address = address;
	GlobalConfig = Config;
}

void Adafruit_BNO055_Begin(adafruit_bno055_opmode_t mode)
{
  uint8_t Data = 0;
  
  /* Make sure we have the right device */
  
  uint8_t id ;
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_CHIP_ID_ADDR , 1 , &id , 1 , HAL_MAX_DELAY);
 
  if (id != BNO055_ID)
  {
    HAL_Delay(1000); // hold on for boot
	
    HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_CHIP_ID_ADDR , 1 , &id , 1 , HAL_MAX_DELAY);
	
    if (id != BNO055_ID)
	{
      return; // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  Adafruit_BNO055_SetMode(OPERATION_MODE_CONFIG);

  /* Reset */
  Data = 0x20;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  HAL_Delay(30);
  
  id = 0x50;
  while (id != BNO055_ID)
  {
    HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_CHIP_ID_ADDR , 1 , &id , 1 , HAL_MAX_DELAY);
    HAL_Delay(10);
  }
  HAL_Delay(50);

  /* Set to normal power mode */
  uint8_t PWR1 = POWER_MODE_NORMAL;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_PWR_MODE_ADDR , 1 , &PWR1 , 1 , HAL_MAX_DELAY);
  HAL_Delay(10);
  
  Data = 0;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_PAGE_ID_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);


  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  HAL_Delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  HAL_Delay(10);
  */

  Data = 0;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  HAL_Delay(10);
  
  /* Set the requested operating mode (see section 3.3) */
  
  Adafruit_BNO055_SetMode(mode);
  HAL_Delay(20);

}

void Adafruit_BNO055_SetMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_OPR_MODE_ADDR , 1 , &_mode , 1 , HAL_MAX_DELAY);
  HAL_Delay(30);
}

int8_t Adafruit_BNO055_GetTemp(void)
{
  int8_t temp;
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_TEMP_ADDR , 1 , &temp , 1 , HAL_MAX_DELAY);
  return temp;
}

void Adafruit_BNO055_SetExtCrystalUse(uint8_t usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;
  uint8_t Data; 
  /* Switch to config mode (just in case since this is the default) */
  Adafruit_BNO055_SetMode(OPERATION_MODE_CONFIG);
  HAL_Delay(25);
  
  Data = 0;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_PAGE_ID_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  
  if (usextal)
  {
    Data = 0x80;
	HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  }
  else
  {
    Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  }
  HAL_Delay(10);
  /* Set the requested operating mode (see section 3.3) */
  Adafruit_BNO055_SetMode(modeback);
  HAL_Delay(20);
}

void Adafruit_BNO055_GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) 
{	
  uint8_t calData;
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_CALIB_STAT_ADDR , 1 , &calData , 1 , HAL_MAX_DELAY);

  if (sys != NULL) 
  {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL)
  {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL)
  {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL)
  {
    *mag = calData & 0x03;
  }
}

void Adafruit_BNO055_GetQuat(Quaternion *quat)
{
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t xt, yt, zt, wt;
  xt = yt = zt = wt = 0;

  /* Read quat data (8 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_QUATERNION_DATA_W_LSB_ADDR , 1 , buffer , 8 , HAL_MAX_DELAY);
  wt = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  xt = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  yt = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  zt = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
  /*!
   * Assign to Quaternion
   * See
   * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  
  quat->w = scale * wt;
  quat->x = scale * xt;
  quat->y = scale * yt;
  quat->z = scale * zt;
 
}

void Adafruit_BNO055_GetVector(adafruit_vector_type_t vector_type , double* xyz)
{
 
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , _address , (adafruit_bno055_reg_t)vector_type , 1 , buffer , 6 , HAL_MAX_DELAY);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type)
  {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  }

}


