

#ifndef SENS_H_
#define SENS_H_

#include "../MainMod.h"

enum {FAST_SCAN,FULL_SCAN};

// zobrazí informace ze senzorů na LCD
void sensInfo();

// zobrazí informace ze senzorů na LCD
void sensFullInfo();

// načte z modulu SensMod
uint8_t getSensorState();




#endif /* SENS_H_ */
