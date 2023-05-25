#ifndef _MPU9250_H
#define _MPU9250_H

typedef enum
{
	A2G,
    A4G,
    A8G,
    A16G
	
}ACCEL_FS_SEL;

typedef enum  
{ 
	G250DPS ,
    G500DPS ,
    G1000DPS,
    G2000DPS
	
}GYRO_FS_SEL;

typedef enum  
{
	M14BITS,
    M16BITS
	
}MAG_OUTPUT_BITS;

typedef enum
{
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ
	
}FIFO_SAMPLE_RATE;

typedef enum
{
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ
	
}GYRO_DLPF_CFG;

typedef enum 
{
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ
	
}ACCEL_DLPF_CFG;

void MPU9250_Init(I2C_HandleTypeDef* Config,ACCEL_FS_SEL AccelRes ,GYRO_FS_SEL GyroRes,
				  FIFO_SAMPLE_RATE Rate,GYRO_DLPF_CFG GCfg,ACCEL_DLPF_CFG ACfg);
void MPU9250_MagnetometerInit(MAG_OUTPUT_BITS MagBits);
void MPU9250_Calibrate_Acc_Gyro(void);
void MPU9250_MainUpdateFunction(void);
int16_t MPU9250_Read_Temperature(void);
void MPU9250_UpdateAccelGyro(void);
void MPU9250_Read_Accel_Gyro(int16_t* destination);
void MPU9250_Calibrate_Mag(void);
void MPU9250_ReadMag(int16_t* destination);
void MPU9250_UpdateMag(void);

float getRoll(void);					
float getPitch(void);    			
float getYaw(void)   ;   			
float getEulerX(void) ;				
float getEulerY(void) 	;			
float getEulerZ(void) 	;			
float getQuaternionX(void);  		
float getQuaternionY(void) ; 		
float getQuaternionZ(void)  ;		
float getQuaternionW(void)  ;		
float getAcc(const uint8_t i); 		
float getGyro(const uint8_t i);  	
float getMag(const uint8_t i) 	;	
float getLinearAcc(const uint8_t i); 
float getAccX(void);   				
float getAccY(void) ;  				
float getAccZ(void)  ; 				
float getGyroX(void) ;				
float getGyroY(void) ; 				
float getGyroZ(void)  ;				
float getMagX(void)   	;			
float getMagY(void)   	;			
float getMagZ(void)   	;			
float getLinearAccX(void); 			
float getLinearAccY(void) ;			
float getLinearAccZ(void) 	;		
float getAccBias(const uint8_t i);   
float getGyroBias(const uint8_t i);  
float getMagBias(const uint8_t i)  ;	
float getMagScale(const uint8_t i)  ;
float getAccBiasX(void);  			
float getAccBiasY(void) ; 			
float getAccBiasZ(void) ; 			
float getGyroBiasX(void) ;			
float getGyroBiasY(void) ;			
float getGyroBiasZ(void) ;			
float getMagBiasX(void)  ;			
float getMagBiasY(void)  ;			
float getMagBiasZ(void)  ;			
float getMagScaleX(void) ;			
float getMagScaleY(void) ;			
float getMagScaleZ(void) ;			
float getTemperature(void);  		



#endif /*_MPU9250_H*/

