#ifndef MODR_H_
#define MODR_H_


#include "../MainMod.h"

// inicializace
void ioinit();

// inicializace regul�toru ujet� vzd.
void initDistReg();

void initAngleReg();

void checkButtons ();

void updateTime();

void writeTime();

void joy();

void standBy();

void set_uarts();

void set_adc();

// spust� AD p�evod pro ur�en� vych�len� joysticku
void update_joystick();

// regul�tor pro ujet� zadan� vzd�lenosti
void distReg();

// regul�tor pro oto�en� o zadan� �hel
void angleReg();

void setAngleReg(int16_t angle);

// nastaven� regul�toru ujet� vzd.
void setDistReg(int16_t dist);

// obsluha lcd - menu
void manageLcd();

// obsluha joysticku
void joyRide();

#endif /* MODR_H_ */
