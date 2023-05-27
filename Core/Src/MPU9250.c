/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#include "MPU9250.h"
#include "spi.h"


const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint8_t _address = 0b11010000;
// 400 kHz
const uint32_t _i2cRate = 400000;

int32_t Accel_x_bias,Accel_y_bias,Accel_z_bias,Gyro_x_bias,Gyro_y_bias,Gyro_z_bias;
float roll,yaw,pitch;
int16_t  MPU9250_FIFO[6][Filter_Buf_SIZE];	//9个FIFO队列；0-2：陀螺仪数据；3-5：加速度计数据；6-8：磁强计数据
static uint8_t Wr_Index = 0;	//当前FIFO的写入下标
// // MPU9250 registers
// const uint8_t ACCEL_OUT = 0x3B;
// const uint8_t GYRO_OUT = 0x43;
// const uint8_t TEMP_OUT = 0x41;
// const uint8_t EXT_SENS_DATA_00 = 0x49;
// const uint8_t ACCEL_CONFIG = 0x1C;
// const uint8_t ACCEL_FS_SEL_2G = 0x00;
// const uint8_t ACCEL_FS_SEL_4G = 0x08;
// const uint8_t ACCEL_FS_SEL_8G = 0x10;
// const uint8_t ACCEL_FS_SEL_16G = 0x18;
// const uint8_t GYRO_CONFIG = 0x1B;
// const uint8_t GYRO_FS_SEL_250DPS = 0x00;
// const uint8_t GYRO_FS_SEL_500DPS = 0x08;
// const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
// const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
// const uint8_t ACCEL_CONFIG2 = 0x1D;
// const uint8_t DLPF_184 = 0x01;
// const uint8_t DLPF_92 = 0x02;
// const uint8_t DLPF_41 = 0x03;
// const uint8_t DLPF_20 = 0x04;
// const uint8_t DLPF_10 = 0x05;
// const uint8_t DLPF_5 = 0x06;
// const uint8_t CONFIG = 0x1A;
// const uint8_t SMPDIV = 0x19;
// const uint8_t INT_PIN_CFG = 0x37;
// const uint8_t INT_ENABLE = 0x38;
// const uint8_t INT_DISABLE = 0x00;
// const uint8_t INT_PULSE_50US = 0x00;
// const uint8_t INT_WOM_EN = 0x40;
// const uint8_t INT_RAW_RDY_EN = 0x01;
// const uint8_t PWR_MGMNT_1 = 0x6B;
// const uint8_t PWR_CYCLE = 0x20;
// const uint8_t PWR_RESET = 0x80;
// const uint8_t CLOCK_SEL_PLL = 0x01;
// const uint8_t PWR_MGMNT_2 = 0x6C;
// const uint8_t SEN_ENABLE = 0x00;
// const uint8_t DIS_GYRO = 0x07;
// const uint8_t USER_CTRL = 0x6A;
// const uint8_t I2C_MST_EN = 0x20;
// const uint8_t I2C_MST_CLK = 0x0D;
// const uint8_t I2C_MST_CTRL = 0x24;
// const uint8_t I2C_SLV0_ADDR = 0x25;
// const uint8_t I2C_SLV0_REG = 0x26;
// const uint8_t I2C_SLV0_DO = 0x63;
// const uint8_t I2C_SLV0_CTRL = 0x27;
// const uint8_t I2C_SLV0_EN = 0x80;
// const uint8_t I2C_READ_FLAG = 0x80;
// const uint8_t MOT_DETECT_CTRL = 0x69;
// const uint8_t ACCEL_INTEL_EN = 0x80;
// const uint8_t ACCEL_INTEL_MODE = 0x40;
// const uint8_t LP_ACCEL_ODR = 0x1E;
// const uint8_t WOM_THR = 0x1F;
// const uint8_t WHO_AM_I = 0x75;
// const uint8_t FIFO_EN = 0x23;
// const uint8_t FIFO_TEMP = 0x80;
// const uint8_t FIFO_GYRO = 0x70;
// const uint8_t FIFO_ACCEL = 0x08;
// const uint8_t FIFO_MAG = 0x01;
// const uint8_t FIFO_COUNT = 0x72;
// const uint8_t FIFO_READ = 0x74;

// // AK8963 registers
// const uint8_t AK8963_I2C_ADDR = 0x0C;
// const uint8_t AK8963_HXL = 0x03;
// const uint8_t AK8963_CNTL1 = 0x0A;
// const uint8_t AK8963_PWR_DOWN = 0x00;
// const uint8_t AK8963_CNT_MEAS1 = 0x12;
// const uint8_t AK8963_CNT_MEAS2 = 0x16;
// const uint8_t AK8963_FUSE_ROM = 0x0F;
// const uint8_t AK8963_CNTL2 = 0x0B;
// const uint8_t AK8963_RESET = 0x01;
// const uint8_t AK8963_ASA = 0x10;
// const uint8_t AK8963_WHO_AM_I = 0x00;


static uint8_t _buffer[21];
static uint8_t _mag_adjust[3];

__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
	MPU9250_OnActivate();
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
	MPU_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	MPU_SPI_Read(dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// store the data for write
	writeRegister(I2C_SLV0_DO,data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I_MPU9250,1,_buffer);

	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(){
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);
	// return the register value
	return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
	// select clock source to gyro
	writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
	// select clock source to gyro
	writeRegister(PWR_MGMT_1,CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI();
	
	if((who != 0x71) &&( who != 0x73))
	{
		return 1;
		//return who;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMT_2, SEN_ENABLE);

	// setting accel range to 2G as default
	writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G);

	// setting the gyro range to 2000DPS as default; Fchoice_b = 0x00, use DLPF.
	writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS);

	// setting bandwidth to 184Hz as default; default accel_fchoice_b = 0x00, use DLPF.
	writeRegister(ACCEL_CONFIG2, DLPF_184);

	// setting gyro bandwidth to 184Hz; Fs = 1kHz.
	writeRegister(CONFIG,DLPF_184);

	// setting the sample rate. Sample rate = 1kHz / (1 + SMPLRT_DIV) = 500Hz; SMPLRT_DIV = 1
	writeRegister(SMPLRT_DIV, 0x01);

	// enable I2C master mode
	writeRegister(USER_CTRL,I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL,7,_buffer);

	// successful init, return 0
	return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
	writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
	writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2,bandwidth);
	writeRegister(CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPLRT_DIV,19);

	if(srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);
	}

	writeRegister(SMPLRT_DIV, srd);
}

/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetRawData(int16_t* AccData, int16_t* MagData, int16_t* GyroData)
{
	// grab the data from the MPU9250
	readRegisters(ACCEL_OUT, 21, _buffer);

	// combine into 16 bit values
	AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	int16_t magx = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	int16_t magy = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	int16_t magz = (((int16_t)_buffer[19]) << 8) | _buffer[18];

	MagData[0] = (int16_t)((float)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
	MagData[1] = (int16_t)((float)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
	MagData[2] = (int16_t)((float)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));
}

void Calibration_IMU()
{
	int16_t AccData_temp[3], GyroData_temp[3], MagData_temp[3];
	int16_t Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z;

	for(int i=0; i < CountOffset; i++){
		MPU9250_GetRawData(AccData_temp, GyroData_temp, MagData_temp);

				/*-------- Accel ---------*/
		Accel_x = (int16_t)AccData_temp[0];
		Accel_y = (int16_t)AccData_temp[1];
		Accel_z = (int16_t)AccData_temp[2];

				/*-------- Gyrometer --------*/
		Gyro_x = (int16_t)GyroData_temp[0];
		Gyro_y = (int16_t)GyroData_temp[1];
		Gyro_z = (int16_t)GyroData_temp[2];

		Accel_x_bias += (int32_t)Accel_x;
		Accel_y_bias += (int32_t)Accel_y;
		Accel_z_bias += (int32_t)Accel_z;

		Gyro_x_bias += (int32_t)Gyro_x;
		Gyro_y_bias += (int32_t)Gyro_y;
		Gyro_z_bias += (int32_t)Gyro_z;
	}
	Accel_x_bias /= CountOffset;
	Accel_y_bias /= CountOffset;
	Accel_z_bias /= CountOffset;

	Gyro_x_bias /= CountOffset;
	Gyro_y_bias /= CountOffset;
	Gyro_z_bias /= CountOffset;

	if(Accel_z_bias > 0) //// Remove gravity from the z-axis accelerometer bias calculation
	{
		Accel_z_bias -= (int32_t)accel_sensitivity;
	}
	else
	{
		Accel_z_bias += (int32_t)accel_sensitivity;
	}
}

static void MPU9250_NewVal(int16_t* buf,int16_t val) {
  	buf[Wr_Index] = val;
}

//计算FIFO中的平均值
static int16_t MPU9250_GetAvg(int16_t* buf)
{
  	int i;
	int32_t	sum = 0;
	for(i=0;i<Filter_Buf_SIZE;i++)
		sum += buf[i];
	sum = sum / Filter_Buf_SIZE;
	return (int16_t)sum;
}

void MPU9250_GetGyro_Acc(int16_t *gyro,int16_t *acc)
{
	// static short buf[6];	//缓存原始数据：0-2：陀螺仪数据；3-5：加速度计数据	
	static int16_t AccData_temp[3], GyroData_temp[3], MagData_temp[3];
	static int16_t gx,gy,gz;
	static int16_t ax,ay,az;
	// static int16_t mx,my,mz;
	
	//正点原子的库函数，读取传感器原始数据
	MPU9250_GetRawData(AccData_temp, GyroData_temp, MagData_temp);
	
	//将原始数据入队
	MPU9250_NewVal(&MPU9250_FIFO[0][0],AccData_temp[0]);
	MPU9250_NewVal(&MPU9250_FIFO[1][0],AccData_temp[1]);
	MPU9250_NewVal(&MPU9250_FIFO[2][0],AccData_temp[2]);

	MPU9250_NewVal(&MPU9250_FIFO[3][0],GyroData_temp[0]);
	MPU9250_NewVal(&MPU9250_FIFO[4][0],GyroData_temp[1]);
	MPU9250_NewVal(&MPU9250_FIFO[5][0],GyroData_temp[2]);

	// MPU9250_NewVal(&MPU9250_FIFO[6][0],MagData_temp[1]);
	// MPU9250_NewVal(&MPU9250_FIFO[7][0],MagData_temp[2]);
	// MPU9250_NewVal(&MPU9250_FIFO[8][0],MagData_temp[3]);
	
	//更新FIFO入口指针
	Wr_Index = (Wr_Index + 1) % Filter_Buf_SIZE;	

	//计算队列平均值
	gyro[0] =  MPU9250_GetAvg(&MPU9250_FIFO[4][0]);
	gyro[1] =  MPU9250_GetAvg(&MPU9250_FIFO[5][0]);
	gyro[2] =  MPU9250_GetAvg(&MPU9250_FIFO[6][0]);
	
	//陀螺仪数据要减去偏移量
	// gyro[0] = gx - Gyro_x_bias;	//gyro
	// gyro[1] = gy - Gyro_y_bias;
	// gyro[2] = gz - Gyro_z_bias;
		

	acc[0] = MPU9250_GetAvg(&MPU9250_FIFO[0][0]);
	acc[1] = MPU9250_GetAvg(&MPU9250_FIFO[1][0]);
	acc[2] = MPU9250_GetAvg(&MPU9250_FIFO[2][0]);
				
	// acc[0] = ax - Accel_x_bias; //acc
	// acc[1] = ay - Accel_y_bias;
	// acc[2] = az - Accel_z_bias;	
}

void Process_IMU()
{
	//read raw data
	uint8_t data[14];
	int16_t gyro[3],acc[3],mag[3];
	//MPU9250_GetGyro_Acc(gyro,acc);
	MPU9250_GetRawData(acc, gyro, mag);
	/*-------- Accel ---------*/
	acc[0] = 9.8*(float)((int32_t)acc[0] - Accel_x_bias)/(float)accel_sensitivity;
	acc[1] = 9.8*(float)((int32_t)acc[1] - Accel_y_bias)/(float)accel_sensitivity;
	acc[2] = 9.8*(float)((int32_t)acc[2] - Accel_z_bias)/(float)accel_sensitivity ;


	/*-------- Gyrometer --------*/
	gyro[0] =  (float)(((int32_t)gyro[0] - Gyro_x_bias)/(float)gyro_sensitivity)*M_PI/180.0f;
	gyro[1] =  (float)(((int32_t)gyro[1] - Gyro_y_bias)/(float)gyro_sensitivity)*M_PI/180.0f;
	gyro[2] =  (float)(((int32_t)gyro[2] - Gyro_z_bias)/(float)gyro_sensitivity)*M_PI/180.0f;

	// Get data of Magnetometer
	//Get_magnetometer();
	//yaw = atan2(Accel_x,Accel_y) * RAD2DEC;
	//new_yaw = get_kalman_angle(yaw,Gyro_z/gyro_sensitivity,0.01);
	//MadgwickAHRSupdateIMU(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2]);
	MahonyAHRSupdateIMU(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2]);
	//MadgwickAHRSupdate(Gyro_X*M_PI/180.0f,Gyro_Y*M_PI/180.0f,Gyro_Z*M_PI/180.0f,Accel_X,Accel_Y,Accel_Z,Mag_X_calib,Mag_Y_calib,-Mag_Z_calib);
	//MadgwickQuaternionUpdate(-Accel_X,Accel_Y,Accel_Z, Gyro_X*M_PI/180.0f,-Gyro_Y*M_PI/180.0f,-Gyro_Z*M_PI/180.0f, Mag_Y_calib,-Mag_X_calib,Mag_Z_calib);
	
	//MahonyAHRSupdate(Gyro_X*M_PI/180.0f,-Gyro_Y*M_PI/180.0f,-Gyro_Z*M_PI/180.0f,Accel_X,-Accel_Y,-Accel_Z,Mag_Y_calib,-Mag_X_calib,Mag_Z_calib);
}

void Quaternion_to_EulerAngle(float w,float x,float y,float z)
{
	  // roll (x-axis rotation)
		float sinr = 2*(w*x + y*z);
		float cosr = 1 - 2*(x*x + y*y);
		roll = atan2(sinr,cosr);

	 // pitch (y-axis rotation)
		float sinp = 2*(w*y - z*x);
		if (abs(sinp) >= 1)
			pitch = copysign(M_PI/2,sinp);
		else
			pitch =  asin(sinp);
	//// yaw (z-axis rotation)
		float siny = 2 *(w*z + x*y);
		float cosy = 1 - 2*(y*y + z*z);
		yaw = atan2(siny,cosy);

		roll = roll*180/M_PI;
		pitch = pitch*180/M_PI;
		yaw = yaw*180/M_PI;
}