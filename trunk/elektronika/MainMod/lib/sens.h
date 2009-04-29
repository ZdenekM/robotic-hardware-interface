

#ifndef SENS_H_
#define SENS_H_

#include "../MainMod.h"



// zobrazí informace ze senzorù na LCD
void sensInfo();

// zobrazí informace ze senzorù na LCD
void sensFullInfo();

// naète z modulu SensMod
void getFastSensorState();

// provede plné skenování a naète data ze SensMod
void getFullSensorState();




#endif /* SENS_H_ */
