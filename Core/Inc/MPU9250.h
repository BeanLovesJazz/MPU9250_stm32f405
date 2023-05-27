/*
 * MPU925.h
 *
 *  Created on: 23 ��� 2018 �.
 *      Author: Max
 */

#ifndef MPU925_H_
#define MPU925_H_

#include "main.h"
#include "MPU9250_Config.h"
#include "Madgwick.h"

#include "math.h"
#include "stdlib.h"

// Magnetometer Register
// #define WHO_AM_I  0x00 // (AKA WIA) should return 0x48
// #define INFO      0x01
// #define ST1       0x02  // data ready status bit 0
// #define XOUT_L    0x03  // data
// #define XOUT_H    0x04
// #define YOUT_L    0x05
// #define YOUT_H    0x06
// #define ZOUT_L    0x07
// #define ZOUT_H    0x08
// #define ST2       0x09  // Data overflow bit 3 and data read error status bit 2
// #define CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
// #define ASTC      0x0C  // Self test control
// #define I2CDIS    0x0F  // I2C disable
// #define ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
// #define ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
// #define ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define AK8963_I2C_ADDR 	0x0C
#define AK8963_HXL		 	0x03
#define AK8963_CNTL1 		0x0A
#define AK8963_PWR_DOWN 	0x00
#define AK8963_CNT_MEAS1 	0x12
#define AK8963_CNT_MEAS2 	0x16
#define AK8963_FUSE_ROM 	0x0F
#define AK8963_CNTL2 		0x0B
#define AK8963_RESET 		0x01
#define AK8963_ASA 			0x10
#define AK8963_WHO_AM_I 	0x00

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_A       0x10

#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250   0x75 // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

#define I2C_SLV0_EN        0x80
#define I2C_READ_FLAG      0x80
#define CLOCK_SEL_PLL	   0x01
#define I2C_MST_EN		   0x20
#define I2C_MST_CLK		   0x0D
#define PWR_RESET		   0x80
#define SEN_ENABLE		   0x00
#define ACCEL_FS_SEL_2G    0x00
#define ACCEL_FS_SEL_4G	   0x08
#define ACCEL_FS_SEL_8G    0x10
#define ACCEL_FS_SEL_16G   0x18
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18
#define DLPF_184 			0x01
#define DLPF_92             0x02
#define DLPF_41             0x03
#define DLPF_20             0x04
#define DLPF_10             0x05
#define DLPF_5              0x06
#define ACCEL_OUT			0x3B
#define GYRO_OUT            0x43
#define TEMP_OUT            0x41
#define CountOffset			100
#define gyro_sensitivity    65.5// 16.4//32.8//65.5//131.0   // =  LSB/degrees/sec //2000//1000//500//250
#define accel_sensitivity  	16384.0//2048.0////4096.0//8192.0//16384.0    // =  LSB/g //16/8/4/2
#define mag_sensitivity    	1.499389499  // Divide raw data by mag_sensitivity to change uT -> mG      raw_Data/(10*4912/32760)
#define Filter_Buf_SIZE  	10
#define M_PI 				3.14159265358979323846

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;

extern int32_t Accel_x_bias,Accel_y_bias,Accel_z_bias,Gyro_x_bias,Gyro_y_bias,Gyro_z_bias;
extern float roll,yaw,pitch;

uint8_t SPIx_WriteRead(uint8_t Byte);
void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void MPU_SPI_Read (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToRead);
void writeRegister(uint8_t subAddress, uint8_t data);
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
void writeAK8963Register(uint8_t subAddress, uint8_t data);
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);


uint8_t MPU9250_Init();
/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetRawData(int16_t* AccData, int16_t* MagData, int16_t* GyroData);

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range);

void Calibration_IMU();

static void MPU9250_NewVal(int16_t* buf,int16_t val);
static int16_t MPU9250_GetAvg(int16_t* buf);
void MPU9250_GetGyro_Acc(int16_t *gyro,int16_t *acc);
void Process_IMU();
void Quaternion_to_EulerAngle(float w,float x,float y,float z);
#endif /* MPU925_H_ */





