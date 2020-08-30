#include "Motors.h"

void Motors_Init(TIM_TypeDef *motorTimer) {
	Motors.TIM = motorTimer;
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

	//situs reiktu keist tik kai stabilizuota jau
	if (z > 0) {
	} else if (z != 0) {
	}

	Motors_Set();
}

void Motors_Set() {
	int16_t temp[4] = { 0 };

	for (uint8_t i = 0, j = 1; i < 4; i++, j += 2) {
		temp[i] = Motors.packet[j] | (Motors.packet[j + 1] << 8);
		if (temp[i] > maxMotorValue) temp[i] = maxMotorValue;
	}

	//todo praleisti per vidurkinima
	if (*(temp) == 0 && *(temp + 1) == 0 && *(temp + 2) == 0 && *(temp + 3) == 0) {
		//kad nesisuktu kai nereikia
		Motors.TIM->CCR1 = minMotorValue;		//BL
		Motors.TIM->CCR2 = minMotorValue;		//FL
		Motors.TIM->CCR3 = minMotorValue;		//BR
		Motors.TIM->CCR4 = minMotorValue;		//FR
		for (uint8_t i = 0; i < 4; i++)
			Motors.coefACC.coeff[i] = 0;
	} else {
		//todo patikrinti min max ribas
		int32_t temp1;

		for (uint8_t i = 0; i < 4; i++) {
			temp1 = temp[i] + Motors.coefACC.coeff[i];
			if (temp1 > maxSiunciamaReiksme) temp1 = maxSiunciamaReiksme;
			if (temp1 < 0) temp1 = 0;
			temp[i] = temp1;
		}

		Motors.TIM->CCR1 = (temp[0] * minMotorValue / maxSiunciamaReiksme + minMotorValue);		//BL
		Motors.TIM->CCR2 = (temp[1] * minMotorValue / maxSiunciamaReiksme + minMotorValue);		//FL
		Motors.TIM->CCR3 = (temp[2] * minMotorValue / maxSiunciamaReiksme + minMotorValue);		//BR
		Motors.TIM->CCR4 = (temp[3] * minMotorValue / maxSiunciamaReiksme + minMotorValue);		//FR
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

