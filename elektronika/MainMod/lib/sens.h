

#ifndef SENS_H_
#define SENS_H_

#include "../MainMod.h"
#include "lcd.h"

// zobrazí informace ze senzorù na LCD
void sensInfo(tsens *s);

// zobrazí informace ze senzorù na LCD
void sensFullInfo(tsens *s);

// naète z modulu SensMod
void getFastSensorState(tcomm_state *c, tsens *s);

// provede plné skenování a naète data ze SensMod
void getFullSensorState(tcomm_state *c, tsens *s);




#endif /* SENS_H_ */
