

#ifndef SENS_H_
#define SENS_H_

#include "../MainMod.h"
#include "lcd.h"

// zobraz� informace ze senzor� na LCD
void sensInfo(tsens *s);

// zobraz� informace ze senzor� na LCD
void sensFullInfo(tsens *s);

// na�te z modulu SensMod
void getFastSensorState(tcomm_state *c, tsens *s);

// provede pln� skenov�n� a na�te data ze SensMod
void getFullSensorState(tcomm_state *c, tsens *s);




#endif /* SENS_H_ */
