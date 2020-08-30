#include "AccGyro.h"

void AG_start(I2C_HandleTypeDef *i2c_instance) {
#if Gyro_FullScale==250
	uint8_t gyroFS = 0;
#elif Gyro_FullScale==500
	uint8_t gyroFS = 1;
#elif Gyro_FullScale==1000
	uint8_t gyroFS = 2;
#elif Gyro_FullScale==2000
	uint8_t gyroFS = 3;
#endif

#if ACC_FullScale==2
	uint8_t accFS=0;
#elif ACC_FullScale==4
	uint8_t accFS=1;
#elif ACC_FullScale==8
	uint8_t accFS = 2;
#elif ACC_FullScale==16
	uint8_t accFS=3;
#endif

	AG.i2c = i2c_instance;

	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);

	//Clock Source
	I2C_Write8(PWR_MAGT_1_REG, 0);
	HAL_Delay(100);  // should wait 10ms after changeing the clock setting.

	//Set the Digital Low Pass Filter
	I2C_Write8(CONFIG_REG, ACC_Filtras);

	//Select the Gyroscope Full Scale Range
	I2C_Write8(GYRO_CONFIG_REG, gyroFS << 3);

	//Select the Accelerometer Full Scale Range

	I2C_Write8(ACCEL_CONFIG_REG, accFS << 3);
	//Set SRD To Default
	I2C_Write8(SMPLRT_DIV_REG, 0x04);

	//clear flag after read
	I2C_Write8(INT_PIN_CFG, (1 << 4) | (1 << 5));

	//enable interupt
	I2C_Write8(INT_ENABLE_REG, 1);
}

void AG_AddNewValues(int16_t x, int16_t y, int16_t z) {
	if (AG.ValueBuffer.Counter < ACC_MeanValues) AG.ValueBuffer.Counter++;

	AG.LastRawValues.x = x;
	AG.LastRawValues.y = y;
	AG.LastRawValues.z = z;

	uint8_t currentPlace = AG.ValueBuffer.Place;

	//atimti seniausias reiksmes
	AG.ValueBuffer.Sum.x -= AG.ValueBuffer.ValueArrays.x[currentPlace];
	AG.ValueBuffer.Sum.y -= AG.ValueBuffer.ValueArrays.y[currentPlace];
	AG.ValueBuffer.Sum.z -= AG.ValueBuffer.ValueArrays.z[currentPlace];

	//irasyti naujausias reiksmes buferyje
	AG.ValueBuffer.ValueArrays.x[currentPlace] = x;
	AG.ValueBuffer.ValueArrays.y[currentPlace] = y;
	AG.ValueBuffer.ValueArrays.z[currentPlace] = z;

	//prideti naujausias reiksmes
	AG.ValueBuffer.Sum.x += x;
	AG.ValueBuffer.Sum.y += y;
	AG.ValueBuffer.Sum.z += z;

	//suskaiciuoti nauja vidurki
	AG.ValueBuffer.CurrentMean.x = (int16_t) ((float) AG.ValueBuffer.Sum.x / (float) AG.ValueBuffer.Counter);
	AG.ValueBuffer.CurrentMean.y = (int16_t) ((float) AG.ValueBuffer.Sum.y / (float) AG.ValueBuffer.Counter);
	AG.ValueBuffer.CurrentMean.z = (int16_t) ((float) AG.ValueBuffer.Sum.z / (float) AG.ValueBuffer.Counter);

	//pakeisti vieta
	AG.ValueBuffer.Place = AG.ValueBuffer.Place == ACC_MeanValues - 1 ? 0 : AG.ValueBuffer.Place + 1;
}

//=================== I2C ===================
void I2C_Write8(uint8_t ADDR, uint8_t data) {
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	HAL_I2C_Master_Transmit(AG.i2c, MPU_ADDR << 1, i2cData, 2, 100);
}
