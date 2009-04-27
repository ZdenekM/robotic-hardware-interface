

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

// spust� AD p�evod pro ur�en� vych�len� joysticku
void update_joystick(tmod_state *m);

// regul�tor pro ujet� zadan� vzd�lenosti
void distReg(tcomm_state *c, tdist_reg *d, tsens *s, tmotors *m);

// regul�tor pro oto�en� o zadan� �hel
void angleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m);

void setAngleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m, int16_t angle);

// nastaven� regul�toru ujet� vzd.
void setDistReg(tdist_reg *d, tmotors *m, int16_t dist);






#endif /* MODR_H_ */
