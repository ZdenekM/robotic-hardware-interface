

#ifndef SENS_H_
#define SENS_H_

#include "../MainMod.h"



// zobraz� informace ze senzor� na LCD
void sensInfo();

// zobraz� informace ze senzor� na LCD
void sensFullInfo();

// na�te z modulu SensMod
void getFastSensorState();

// provede pln� skenov�n� a na�te data ze SensMod
void getFullSensorState();




#endif /* SENS_H_ */
