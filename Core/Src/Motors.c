#include "Motors.h"

void Motors_Init(TIM_TypeDef *motorTimer) {
	Motors.TIM = motorTimer;

}
