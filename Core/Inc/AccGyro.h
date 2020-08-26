#ifndef INC_ACCGYRO_H_
#define INC_ACCGYRO_H_

#include "main.h"

#define INT_PIN_CFG 		0x37
#define INT_ENABLE_REG 		0x38

struct {
	I2C_HandleTypeDef *i2c;
} AG;

void AG_init(I2C_HandleTypeDef *i2c_instance);

#endif /* INC_ACCGYRO_H_ */
