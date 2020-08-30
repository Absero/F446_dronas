#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//Includes
#include "main.h"
#include "settings.h"
#include "AccGyro.h"

//defines

//Structs
struct {
	TIM_TypeDef *TIM;
	uint8_t packet[10];
	struct {
		int16_t coeff[4];
		uint8_t addedFR :1;
		uint8_t addedFL :1;
		uint8_t unused :6;
	} coefACC;
} Motors;

enum {
	BL = 0, FL, BR, FR
};

//Functions
void Motors_Init(TIM_TypeDef *motorTimer);

void Motors_Hold();

void Motors_Set();

void tiltFL(int8_t kryptis);
void tiltFR(int8_t kryptis);

#endif /* INC_MOTORS_H_ */
