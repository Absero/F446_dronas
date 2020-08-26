#include "AccGyro.h"

void AG_init(I2C_HandleTypeDef *i2c_instance) {
	AG.i2c = i2c_instance;
}
