

#ifndef MODR_H_
#define MODR_H_

#include "../MainMod.h"
#include "lcd.h"
#include "motors.h"

void checkButtons (tmod_state *m);

void updateTime(tmod_state *m);

void writeTime(tmod_state *m);

void joy(tmod_state *m);

void standBy();

void set_uarts();

void set_adc(void);

// spustí AD pøevod pro urèení vychýlení joysticku
void update_joystick(tmod_state *m);

// regulátor pro ujetí zadané vzdálenosti
void distReg(tcomm_state *c, tdist_reg *d, tsens *s, tmotors *m);

// regulátor pro otoèení o zadaný úhel
void angleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m);

void setAngleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m, int16_t angle);

// nastavení regulátoru ujeté vzd.
void setDistReg(tdist_reg *d, tmotors *m, int16_t dist);






#endif /* MODR_H_ */
