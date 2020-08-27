#ifndef INC_ACCGYRO_H_
#define INC_ACCGYRO_H_

//Includes
#include "main.h"
#include "settings.h"

//Misc defines
#define OneG (32768.0f / ACC_FullScale)
#define ACC_LSB_VALUE (ACC_FullScale * 1000.0f / 32768.0f)

//Address
#define MPU_ADDR			0x68

//Define Registers
#define WHO_AM_I_REG		0x75
#define PWR_MAGT_1_REG		0x6B
#define CONFIG_REG			0x1A
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C
#define SMPLRT_DIV_REG		0x19
#define INT_STATUS_REG		0x3A
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define FIFO_EN_REG 		0x23
#define I2CMACO_REG 		0x23
#define USER_CNT_REG		0x6A
#define FIFO_COUNTH_REG 	0x72
#define FIFO_R_W_REG 		0x74

#define INT_PIN_CFG 		0x37
#define INT_ENABLE_REG 		0x38

//Type definitions
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} RawData_t;

//Structures
struct {
	I2C_HandleTypeDef *i2c;
	RawData_t LastRawValues;
	struct {
		struct {
			int16_t x[ACC_MeanValues];
			int16_t y[ACC_MeanValues];
			int16_t z[ACC_MeanValues];
		} ValueArrays;
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		} CurrentMean;
		struct {
			int32_t x;
			int32_t y;
			int32_t z;
		} Sum;
		uint8_t Place;	//Place in array
		uint8_t Counter;
	} ValueBuffer;

} AG;

//Functions
void AG_start(I2C_HandleTypeDef *i2c_instance);

void AG_AddNewValues(uint16_t x, uint16_t y, uint16_t z);

//=================== I2C ===================
void I2C_Write8(uint8_t ADDR, uint8_t data);

#endif /* INC_ACCGYRO_H_ */
