#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#define maxSiunciamaReiksme 500.0f
#define minMotorValue 500.0f
#define maxMotorValue maxSiunciamaReiksme + minMotorValue

#define Gyro_FullScale  1000		//250, 500, 1000, 2000 (laipsniais/sekunde)
#define ACC_FullScale  8			//2, 4, 8, 16	(g)
#define ACC_Filtras 0 				//Gali but 0-6, didinant reiksme siaureja LP filtro juosta bet dideja velinimas
#define ACC_MeanValues 50			//Is kiek reiksmiu vidurkint
#define ACC_TimkamosReiksmesDiapazonas 100	//Diapazonas, kuriame akselerometro reiksmes priimamos kaip tinkamos
#define Motoru_Coef_Limit (maxSiunciamaReiksme/4)

#endif /* INC_SETTINGS_H_ */
