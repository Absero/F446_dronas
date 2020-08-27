#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//Includes
#include "main.h"

//Structs
struct {
	TIM_TypeDef *TIM;
} Motors;

//Functions
void Motors_Init(TIM_TypeDef *motorTimer);

#endif /* INC_MOTORS_H_ */
