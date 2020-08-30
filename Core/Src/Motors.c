#include "Motors.h"

void Motors_Init(TIM_TypeDef *motorTimer) {
	Motors.TIM = motorTimer;
}

void Motors_ChangeValueFromPacket() {
	int16_t temp[4] = { 0 };

	for (uint8_t i = 0, j = 1; i < 4; i++, j += 2) {
		temp[i] = Motors.packet[j] | (Motors.packet[j + 1] << 8);
		if (temp[i] > maxMotorValue) temp[i] = maxMotorValue;
	}

	Motors_Set(temp);
}

void Motors_Hold() {
	//=========== Apskaiciuoti koeficientus islyginimui ===========
	//Uzfixuoti reiksmes
	int16_t x = AG.ValueBuffer.CurrentMean.x, y = AG.ValueBuffer.CurrentMean.y, z = AG.ValueBuffer.CurrentMean.z;

	/*
	 * 		*** x asis nukreipta i FR, y asis nukreipta i FL ***
	 * 		reiktu patikrint ar Y teigiamas, kad nedarytu nesamoniu apsivertes
	 */

	//Apskaiciuot koeficientus
	if (x > ACC_TimkamosReiksmesDiapazonas) tiltFR(-1);
	else if (x < -ACC_TimkamosReiksmesDiapazonas) tiltFR(1);
	if (y > ACC_TimkamosReiksmesDiapazonas) tiltFL(-1);
	else if (y < -ACC_TimkamosReiksmesDiapazonas) tiltFL(1);

	if (z > 0) {
	} else if (z != 0) {
	}

	int16_t temp[4] = { 0 };

	for (uint8_t i = 0, j = 1; i < 4; i++, j += 2) {
		temp[i] = Motors.packet[j] | (Motors.packet[j + 1] << 8);
		if (temp[i] > maxMotorValue) temp[i] = maxMotorValue;
	}

	Motors_Set(temp);
}

void Motors_Set(int16_t *valueArray) {
	//todo perkelti masyvo sudaryma i cia
	int16_t temp[4] = { 0 };

	for (uint8_t i = 0, j = 1; i < 4; i++, j += 2) {
		temp[i] = Motors.packet[j] | (Motors.packet[j + 1] << 8);
		if (temp[i] > maxMotorValue) temp[i] = maxMotorValue;
	}

	//todo praleisti per vidurkinima
	if (*(valueArray) == 0 && *(valueArray + 1) == 0 && *(valueArray + 2) == 0 && *(valueArray + 3) == 0) {
		//kad nesisuktu kai nereikia
		Motors.TIM->CCR1 = (*valueArray * minMotorValue / maxSiunciamaReiksme + minMotorValue);			//BL
		Motors.TIM->CCR2 = (*(valueArray + 1) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//FL
		Motors.TIM->CCR3 = (*(valueArray + 2) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//BR
		Motors.TIM->CCR4 = (*(valueArray + 3) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//FR
		for (uint8_t i = 0; i < 4; i++)
			Motors.coefACC.coeff[i] = 0;
	} else {
		//todo patikrinti min max ribas
		Motors.TIM->CCR1 = ((*valueArray + Motors.coefACC.coeff[0]) * minMotorValue / maxSiunciamaReiksme + minMotorValue);			//BL
		Motors.TIM->CCR2 = ((*(valueArray + 1) + Motors.coefACC.coeff[1]) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//FL
		Motors.TIM->CCR3 = ((*(valueArray + 2) + Motors.coefACC.coeff[2]) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//BR
		Motors.TIM->CCR4 = ((*(valueArray + 3) + Motors.coefACC.coeff[3]) * minMotorValue / maxSiunciamaReiksme + minMotorValue);	//FR
	}
}

void tiltFL(int8_t kryptis) {
	//kai teigiamas - FL kyla i virsu
	if (Motors.coefACC.addedFL) {
		Motors.coefACC.addedFL = 0;
		if (Motors.coefACC.coeff[1] + 2 * kryptis < Motoru_Coef_Limit && Motors.coefACC.coeff[1] + 2 * kryptis > -Motoru_Coef_Limit)
			Motors.coefACC.coeff[1] += kryptis;
	} else {
		Motors.coefACC.addedFL = 1;
		if (Motors.coefACC.coeff[2] - 2 * kryptis < Motoru_Coef_Limit && Motors.coefACC.coeff[2] - 2 * kryptis > -Motoru_Coef_Limit)
			Motors.coefACC.coeff[2] -= kryptis;
	}
}

void tiltFR(int8_t kryptis) {
	//kai teigiamas - FR kyla i virsu
	if (Motors.coefACC.addedFR) {
		Motors.coefACC.addedFR = 0;
		if (Motors.coefACC.coeff[3] + 2 * kryptis < Motoru_Coef_Limit && Motors.coefACC.coeff[3] + 2 * kryptis > -Motoru_Coef_Limit)
			Motors.coefACC.coeff[3] += kryptis;
	} else {
		Motors.coefACC.addedFR = 1;
		if (Motors.coefACC.coeff[0] - 2 * kryptis < Motoru_Coef_Limit && Motors.coefACC.coeff[0] - 2 * kryptis > -Motoru_Coef_Limit)
			Motors.coefACC.coeff[0] -= kryptis;
	}
}

