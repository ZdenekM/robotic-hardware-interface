#ifndef MODR_H_
#define MODR_H_


#include "../MainMod.h"

// inicializace
void ioinit();

// inicializace regulátoru ujeté vzd.
void initDistReg();

void initAngleReg();

void checkButtons ();

void updateTime();

void writeTime();

void joy();

void standBy();

void set_uarts();

void set_adc();

// spustí AD prevod pro urceni vychyleni joysticku
void update_joystick();

// regulátor pro ujetí zadané vzdálenosti
void distReg();

// regulátor pro otoceni o zadany uhel
void angleReg();

void setAngleReg(int16_t angle);

// nastavení regulátoru ujeté vzd.
void setDistReg(int16_t dist);

// obsluha lcd - menu
void manageLcd();

// obsluha joysticku
void joyRide();

#endif /* MODR_H_ */
