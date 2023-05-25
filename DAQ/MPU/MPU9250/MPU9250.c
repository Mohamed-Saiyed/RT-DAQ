
#include "MPU9250.h"
#include "MPU9250RegisterMap.h"

static uint8_t MPU9250_WHOAMI_DEFAULT_VALUE{0x71};
static uint8_t MPU9255_WHOAMI_DEFAULT_VALUE{0x73};

static constexpr uint8_t MPU9250_DEFAULT_ADDRESS{0x68};  // Device address when ADO = 0
static constexpr uint8_t AK8963_ADDRESS{0x0C};           //  Address of magnetometer

static  uint8_t AK8963_WHOAMI_DEFAULT_VALUE{0x48};
static uint8_t MPU9250_ADDRESS{MPU9250_DEFAULT_ADDRESS};

// settingsk
static uint8_t MAG_MODE{0x06};  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
static float acc_resolution;                     // scale resolutions per LSB for the sensors
static float gyro_resolution;                    // scale resolutions per LSB for the sensors
static float mag_resolution;                     // scale resolutions per LSB for the sensors

// Calibration Parameters
static float acc_bias[3]{0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
static float gyro_bias[3]{0., 0., 0.};  // gyro calibration value in GYRO_FS_SEL: 250dps
static float mag_bias_factory[3]{0., 0., 0.};
static float mag_bias[3]{0., 0., 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
static float mag_scale[3]{1., 1., 1.};
static float magnetic_declination = +4.63;  // Cairo, 18th Feb

static uint16_t CALIB_GYRO_SENSITIVITY = 131;     // = 131 LSB/degrees/sec
static uint16_t CALIB_ACCEL_SENSITIVITY = 16384;  // = 16384 LSB/g

// Temperature
static int16_t temperature_count;  // temperature raw count output
static float temperature;          // Stores the real internal chip temperature in degrees Celsius

// Self Test
static float self_test_result[6];  // holds results of gyro and accelerometer self test



// IMU Data
static float a[3], g[3], m[3];
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
static float euler[3] = {0.f};
static float lin_acc[3];  // linear acceleration (acceleration with gravity component subtracted)

static uint8_t gyro_fchoice{0x03};
static uint8_t accel_fchoice{0x01};

//QuaternionFilter quat_filter;

// Other settings
//bool b_ahrs{true};
//bool b_verbose{false};


/*pointer to save sensor I2c configrations prameters*/
static I2C_HandleTypeDef* GlobalConfig = NULL;
		
void MPU9250_Init(I2C_HandleTypeDef* Config,ACCEL_FS_SEL AccelRes ,GYRO_FS_SEL GyroRes,
				  FIFO_SAMPLE_RATE Rate,GYRO_DLPF_CFG GCfg,ACCEL_DLPF_CFG ACfg)
{
	uint8_t Data;
	acc_resolution 	= get_acc_resolution(AccelRes);
	gyro_resolution = get_gyro_resolution(GyroRes);
	
	GlobalConfig = Config;
	
	/*wake up device*/
	Data = 0x00 ;		
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);

	/* get stable time source */
	Data = 0x01;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);
	
	Data = (uint8_t)GCfg;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , MPU_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY);
	
	Data = (uint8_t)Rate;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , SMPLRT_DIV , 1 , &Data , 1 , HAL_MAX_DELAY);
	
	uint8_t c;
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , GYRO_CONFIG , 1 , &c , 1 , HAL_MAX_DELAY);
	
	c = c & ~0xE0;                                        // Clear self-test bits [7:5]
    c = c & ~0x03;                                        // Clear Fchoice bits [1:0]
    c = c & ~0x18;                                        // Clear GYRO_FS_SEL bits [4:3]
    c = c | (uint8_t(GyroRes) << 3);          // Set full scale range for the gyro
    c = c | (uint8_t(gyro_fchoice) & 0x03);       // Set Fchoice for the gyro
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , SMPLRT_DIV , 1 , &c , 1 , HAL_MAX_DELAY);
	
    //Set accelerometer full-scale range configuration
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , ACCEL_CONFIG , 1 , &c , 1 , HAL_MAX_DELAY);
    c = c & ~0xE0;                                 // Clear self-test bits [7:5]
    c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
    c = c | (uint8_t(AccelRes) << 3);  // Set full scale range for the accelerometer
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , ACCEL_CONFIG , 1 , &c , 1 , HAL_MAX_DELAY);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , ACCEL_CONFIG2 , 1 , &c , 1 , HAL_MAX_DELAY);
    c = c & ~0x0F;                                           // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | ((uint8_t(accel_fchoice) & 0x01) << 3);  // Set accel_fchoice_b to 1
    c = c | uint8_t(ACfg);                 // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , ACCEL_CONFIG2 , 1 , &c , 1 , HAL_MAX_DELAY);

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //write_byte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    //write_byte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    //delay(100);
	
}

void MPU9250_MagnetometerInit(MAG_OUTPUT_BITS MagBits)
{
		uint8_t Data;
	
        uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
		
		mag_resolution 	= get_mag_resolution(MagBits);
		
		 // First extract the factory calibration for each magnetometer axis
		Data = 0x00;
		HAL_I2C_Mem_Write(GlobalConfig , AK8963_ADDRESS , AK8963_CNTL , 1 , &Data , 1 , HAL_MAX_DELAY);
		
		Data = 0x0F;
		HAL_I2C_Mem_Write(GlobalConfig , AK8963_ADDRESS , AK8963_CNTL , 1 , &Data , 1 , HAL_MAX_DELAY);// Enter Fuse ROM access mode
		
		HAL_I2C_Mem_Read(GlobalConfig , AK8963_ADDRESS , AK8963_ASAX , 1 , raw_data , 3 , HAL_MAX_DELAY); // Read the x-, y-, and z-axis calibration values
        mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
        mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
        mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
		
		Data = 0x00;
		HAL_I2C_Mem_Write(GlobalConfig , AK8963_ADDRESS , AK8963_CNTL , 1 , &Data , 1 , HAL_MAX_DELAY);// Power down magnetometer

        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		Data = (uint8_t)MagBits << 4 | MAG_MODE;
		HAL_I2C_Mem_Write(GlobalConfig , AK8963_ADDRESS , AK8963_CNTL , 1 , &Data , 1 , HAL_MAX_DELAY);// Set magnetometer data resolution and sample ODR
               
}

void MPU9250_MainUpdateFunction(void)
{
	
}
void MPU9250_Calibrate_Acc_Gyro(void)
{
    set_acc_gyro_to_calibration();
    collect_acc_gyro_data_to(acc_bias, gyro_bias);
    write_accel_offset();
    write_gyro_offset();
}

static void set_acc_gyro_to_calibration(void)
{
	uint8_t Data;
	
	// reset device
	Data = 0x80;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);// Write a one to bit 7 reset bit; toggle reset device
	
	
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
	Data = 0x01;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_2 , 1 , &Data , 1 , HAL_MAX_DELAY);
	
	// Configure device for bias calculation
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , INT_ENABLE , 1 , &Data , 1 , HAL_MAX_DELAY);// Disable all interrupts
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , FIFO_EN , 1 , &Data , 1 , HAL_MAX_DELAY);// Disable FIFO
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);// Turn on internal clock source
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , I2C_MST_CTRL , 1 , &Data , 1 , HAL_MAX_DELAY);// Disable I2C master
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , USER_CTRL , 1 , &Data , 1 , HAL_MAX_DELAY);// Disable FIFO and I2C master modes
	
	Data = 0x0C;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , USER_CTRL , 1 , &Data , 1 , HAL_MAX_DELAY); // Reset FIFO and DMP
	
	 // Configure MPU6050 gyro and accelerometer for bias calculation
	Data = 0x01;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , MPU_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY); // Set low-pass filter to 188 Hz
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , SMPLRT_DIV , 1 , &Data , 1 , HAL_MAX_DELAY);// Set sample rate to 1 kHz
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , GYRO_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , ACCEL_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY);// Set accelerometer full-scale to 2 g, maximum sensitivity
	
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	Data = 0x40;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , USER_CTRL , 1 , &Data , 1 , HAL_MAX_DELAY);// Enable FIFO
	
	Data = 0x78;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , FIFO_EN , 1 , &Data , 1 , HAL_MAX_DELAY);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	
	
	HAL_Delay(40);// accumulate 40 samples in 40 milliseconds = 480 bytes	
}


static void collect_acc_gyro_data_to(float* a_bias, float* g_bias)
{
	// At end of sample accumulation, turn off FIFO sensor read
	uint8_t data[12];// data array to hold accelerometer and gyro x, y, z, data
	uint8_t Data;
	
	Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS , FIFO_EN , 1 , &Data , 1 , HAL_MAX_DELAY); // Disable gyro and accelerometer sensors for FIFO
	
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , FIFO_COUNTH , 1 , data , 2 , HAL_MAX_DELAY);  // read FIFO sample count
	
	uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging
	
	for (uint16_t ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		
		HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , FIFO_R_W , 1 , data , 12 , HAL_MAX_DELAY);// read data for averaging
		
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		a_bias[1] += (float)accel_temp[1];
		a_bias[2] += (float)accel_temp[2];
		g_bias[0] += (float)gyro_temp[0];
		g_bias[1] += (float)gyro_temp[1];
		g_bias[2] += (float)gyro_temp[2];
		
	}
	
	a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
	a_bias[1] /= (float)packet_count;
	a_bias[2] /= (float)packet_count;
	g_bias[0] /= (float)packet_count;
	g_bias[1] /= (float)packet_count;
	g_bias[2] /= (float)packet_count;

	if (a_bias[2] > 0L)
	{
		a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else
	{
		a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
	}
}


static void write_accel_offset(void)
{
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
	
	uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};                         // A place to hold the factory accelerometer trim biases
	int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
	uint8_t write_data[6] = {0};
	 
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , XA_OFFSET_H , 1 , read_data , 2 , HAL_MAX_DELAY); // Read factory accelerometer trim values
    acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
	
    HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , YA_OFFSET_H , 1 , read_data , 2 , HAL_MAX_DELAY);
    acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
	
    HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , ZA_OFFSET_H , 1 , read_data , 2 , HAL_MAX_DELAY);
    acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];
	
	for (int i = 0; i < 3; i++)
	{
		if (acc_bias_reg[i] % 2)
		{
			mask_bit[i] = 0;
		}
		acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
		if (mask_bit[i])
		{
			acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
		} 
		else
		{
			acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
		}
	}
	
	write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
	write_data[1] = (acc_bias_reg[0]) & 0xFF;
	write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
	write_data[3] = (acc_bias_reg[1]) & 0xFF;
	write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
	write_data[5] = (acc_bias_reg[2]) & 0xFF;
	
	// Push accelerometer biases to hardware registers
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, XA_OFFSET_H, 1 , &write_data[0], 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, XA_OFFSET_L, 1 , &write_data[1], 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, YA_OFFSET_H, 1 , &write_data[2], 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, YA_OFFSET_L, 1 , &write_data[3], 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, ZA_OFFSET_H, 1 , &write_data[4], 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(GlobalConfig , MPU9250_ADDRESS, ZA_OFFSET_L, 1 , &write_data[5], 1 , HAL_MAX_DELAY);
		
}

static void write_gyro_offset(void)
{
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    uint8_t gyro_offset_data[6]{0};
    gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
    gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;
	
	// Push gyro biases to hardware registers
    write_byte(GlobalConfig , MPU9250_ADDRESS, XG_OFFSET_H, 1 , &gyro_offset_data[0], 1 , HAL_MAX_DELAY);
    write_byte(GlobalConfig , MPU9250_ADDRESS, XG_OFFSET_L, 1 , &gyro_offset_data[1], 1 , HAL_MAX_DELAY);
    write_byte(GlobalConfig , MPU9250_ADDRESS, YG_OFFSET_H, 1 , &gyro_offset_data[2], 1 , HAL_MAX_DELAY);
    write_byte(GlobalConfig , MPU9250_ADDRESS, YG_OFFSET_L, 1 , &gyro_offset_data[3], 1 , HAL_MAX_DELAY);
    write_byte(GlobalConfig , MPU9250_ADDRESS, ZG_OFFSET_H, 1 , &gyro_offset_data[4], 1 , HAL_MAX_DELAY);
    write_byte(GlobalConfig , MPU9250_ADDRESS, ZG_OFFSET_L, 1 , &gyro_offset_data[5], 1 , HAL_MAX_DELAY);

	
}

void MPU9250_Calibrate_Mag(void)
{
	MAG_OUTPUT_BITS mag_output_bits_cache = mag_resolution;
	
	MPU9250_MagnetometerInit(M16BITS);
	collect_mag_data_to(mag_bias, mag_scale);
	
	// restore MAG_OUTPUT_BITS
	//mag_output_bits = mag_output_bits_cache;
	//initAK8963();

}

static void collect_mag_data_to(float* m_bias, float* m_scale) 
{
	// shoot for ~fifteen seconds of mag data
	int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_temp[3] = {0, 0, 0};
	uint16_t sample_count = 0;
	
	if (MAG_MODE == 0x02)
	{
		sample_count = 128;     // at 8 Hz ODR, new mag data is available every 125 ms
	}
	else if (MAG_MODE == 0x06)  // in this library, fixed to 100Hz
	{
		sample_count = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms
	}
	else
	{
		
	}
	
	for (uint16_t ii = 0; ii < sample_count; ii++)
	{
		MPU9250_ReadMag(mag_temp);  // Read the mag data
		
		for (int jj = 0; jj < 3; jj++)
		{
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if (MAG_MODE == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if (MAG_MODE == 0x06) HAL_Delay(12);   // at 100 Hz ODR, new mag data is available every 10 ms
    }
	
	// Get hard iron correction
	bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	float bias_resolution = get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
	m_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0];  // save mag biases in G for main program
	m_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
	m_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];

	// Get soft iron correction estimate
	scale[0] = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
	scale[1] = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
	scale[2] = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

	float avg_rad = scale[0] + scale[1] + scale[2];
	avg_rad /= 3.0;

	m_scale[0] = avg_rad / ((float)scale[0]);
	m_scale[1] = avg_rad / ((float)scale[1]);
	m_scale[2] = avg_rad / ((float)scale[2]);
	
}
   
void MPU9250_UpdateMag(void) 
{
    int16_t mag_count[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output
    MPU9250_ReadMag(mag_count);               // Read the x/y/z adc values
    // get_mag_resolution();

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    // mag_bias is calcurated in 16BITS
    float bias_to_current_bits = mag_resolution / get_mag_resolution(M16BITS);
    m[0] = (float)(mag_count[0] * mag_resolution * mag_bias_factory[0] - mag_bias[0] * bias_to_current_bits) * mag_scale[0];  // get actual magnetometer value, this depends on scale being set
    m[1] = (float)(mag_count[1] * mag_resolution * mag_bias_factory[1] - mag_bias[1] * bias_to_current_bits) * mag_scale[1];
    m[2] = (float)(mag_count[2] * mag_resolution * mag_bias_factory[2] - mag_bias[2] * bias_to_current_bits) * mag_scale[2];
}

void MPU9250_ReadMag(int16_t* destination)
{
	uint8_t raw_data[7];// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t data;
	
	HAL_I2C_Mem_Read(GlobalConfig , AK8963_ADDRESS , AK8963_ST1 , 1 , data , 1 , HAL_MAX_DELAY);
	if (data & 0x01)// wait for magnetometer data ready bit to be set
	{     
		HAL_I2C_Mem_Read(GlobalConfig , AK8963_ADDRESS , AK8963_XOUT_L , 1 , raw_data , 7 , HAL_MAX_DELAY); // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
		if (!(c & 0x08))												 // Check if magnetic sensor overflow set, if not then report data
		{                                               
			destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
			destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
		}
	}
}

void MPU9250_UpdateAccelGyro(void)
{
	int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
	read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read

	// Now we'll calculate the accleration value into actual g's
	a[0] = (float)raw_acc_gyro_data[0] * acc_resolution;  // get actual g value, this depends on scale being set
	a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
	a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

	temperature_count = raw_acc_gyro_data[3];                  // Read the adc values
	temperature = ((float)temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

	// Calculate the gyro value into actual degrees per second
	g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution;  // get actual gyro value, this depends on scale being set
	g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
	g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;
}

void MPU9250_Read_Accel_Gyro(int16_t* destination)
{
	uint8_t raw_data[14];                                         // x/y/z accel register data stored here
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , ACCEL_XOUT_H , 1 , raw_data , 14 , HAL_MAX_DELAY);// Read the 14 raw data registers into data array
	destination[0] = ((int16_t)raw_data[0] << 8) | raw_data[1];   // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)raw_data[2] << 8) | raw_data[3];
	destination[2] = ((int16_t)raw_data[4] << 8) | raw_data[5];
	destination[3] = ((int16_t)raw_data[6] << 8) | raw_data[7];
	destination[4] = ((int16_t)raw_data[8] << 8) | raw_data[9];
	destination[5] = ((int16_t)raw_data[10] << 8) | raw_data[11];
	destination[6] = ((int16_t)raw_data[12] << 8) | raw_data[13];
}

int16_t MPU9250_Read_Temperature(void)
{
	uint8_t raw_data[2];                                       // x/y/z gyro register data stored here
	
	HAL_I2C_Mem_Read(GlobalConfig , MPU9250_ADDRESS , TEMP_OUT_H , 1 , raw_data , 2 , HAL_MAX_DELAY); // Read the two raw data registers sequentially into data array
	return ((int16_t)raw_data[0] << 8) | raw_data[1];          // Turn the MSB and LSB into a 16-bit value
}
	
static float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits)
{
    switch (mag_output_bits) 
	{
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        // Proper scale to return milliGauss
        case MAG_OUTPUT_BITS::M14BITS:
            return 10. * 4912. / 8190.0;
        case MAG_OUTPUT_BITS::M16BITS:
            return 10. * 4912. / 32760.0;
        default:
            return 0.;
    }
}	

static float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel)
{
	switch (accel_af_sel) 
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case ACCEL_FS_SEL::A2G:
			return 2.0 / 32768.0;
		case ACCEL_FS_SEL::A4G:
			return 4.0 / 32768.0;
		case ACCEL_FS_SEL::A8G:
			return 8.0 / 32768.0;
		case ACCEL_FS_SEL::A16G:
			return 16.0 / 32768.0;
		default:
			return 0.;
	}
}

static float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel)
{
	switch (gyro_fs_sel) 
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GYRO_FS_SEL::G250DPS:
			return 250.0 / 32768.0;
		case GYRO_FS_SEL::G500DPS:
			return 500.0 / 32768.0;
		case GYRO_FS_SEL::G1000DPS:
			return 1000.0 / 32768.0;
		case GYRO_FS_SEL::G2000DPS:
			return 2000.0 / 32768.0;
		default:
			return 0.;
	}
}

float getRoll(void)					 { return euler[0]; }
float getPitch(void)    			 { return -euler[1]; }
float getYaw(void)      			 { return -euler[2]; }
float getEulerX(void) 				 { return euler[0]; }
float getEulerY(void) 				 { return euler[1]; }
float getEulerZ(void) 				 { return euler[2]; }
float getQuaternionX(void)  		 { return q[1]; }
float getQuaternionY(void)  		 { return q[2]; }
float getQuaternionZ(void)  		 { return q[3]; }
float getQuaternionW(void)  		 { return q[0]; }
float getAcc(const uint8_t i) 		 { return (i < 3) ? a[i] : 0.f; }
float getGyro(const uint8_t i)  	 { return (i < 3) ? g[i] : 0.f; }
float getMag(const uint8_t i) 		 { return (i < 3) ? m[i] : 0.f; }
float getLinearAcc(const uint8_t i)  { return (i < 3) ? lin_acc[i] : 0.f; }
float getAccX(void)   				 { return a[0]; }
float getAccY(void)   				 { return a[1]; }
float getAccZ(void)   				 { return a[2]; }
float getGyroX(void)  				 { return g[0]; }
float getGyroY(void)  				 { return g[1]; }
float getGyroZ(void)  				 { return g[2]; }
float getMagX(void)   				 { return m[0]; }
float getMagY(void)   				 { return m[1]; }
float getMagZ(void)   				 { return m[2]; }
float getLinearAccX(void) 			 { return lin_acc[0]; }
float getLinearAccY(void) 			 { return lin_acc[1]; }
float getLinearAccZ(void) 			 { return lin_acc[2]; }
float getAccBias(const uint8_t i)    { return (i < 3) ? acc_bias[i] : 0.f; }
float getGyroBias(const uint8_t i)   { return (i < 3) ? gyro_bias[i] : 0.f; }
float getMagBias(const uint8_t i)  	 { return (i < 3) ? mag_bias[i] : 0.f; }
float getMagScale(const uint8_t i)   { return (i < 3) ? mag_scale[i] : 0.f; }
float getAccBiasX(void)  			 { return acc_bias[0]; }
float getAccBiasY(void)  			 { return acc_bias[1]; }
float getAccBiasZ(void)  			 { return acc_bias[2]; }
float getGyroBiasX(void) 			 { return gyro_bias[0]; }
float getGyroBiasY(void) 			 { return gyro_bias[1]; }
float getGyroBiasZ(void) 			 { return gyro_bias[2]; }
float getMagBiasX(void)  			 { return mag_bias[0]; }
float getMagBiasY(void)  			 { return mag_bias[1]; }
float getMagBiasZ(void)  			 { return mag_bias[2]; }
float getMagScaleX(void) 			 { return mag_scale[0]; }
float getMagScaleY(void) 			 { return mag_scale[1]; }
float getMagScaleZ(void) 			 { return mag_scale[2]; }
float getTemperature(void)  		 { return temperature; }
