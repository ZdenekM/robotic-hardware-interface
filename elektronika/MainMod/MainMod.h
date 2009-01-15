#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// data odes�lan� do PC
enum {COMMSTATE} tpcdata;

// stav modulu - menu, tla��tka, �as atd.
typedef struct {

	// stavy menu
	enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_JOYSTICK} tmenu_states;

	// aktu�ln� stav menu
	volatile uint8_t menu_state;

	// po��tadlo pro kr�tk� zapnut� podsv�tlen� po stisku tla��tka
	volatile uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	// stav joysticku
	volatile uint16_t joy_x, joy_y;

	// stav tla��tek - po p�e�ten� vynulovat
	uint8_t buttons;
	#define ABUTT1 0
	#define ABUTT2 1
	#define ABUTT3 2
	#define ABUTT4 3

} tmod_state;


extern inline void set_uarts();

extern inline void set_adc(void);

extern uint16_t moving_average_x(uint16_t value);

extern uint16_t moving_average_y(uint16_t value);

extern uint16_t joystick_xy(uint8_t dir);

extern inline void ioinit(void);

extern void checkButtons ();

extern void updateTime();

extern char *getTimeString();

// vyp�e �as na lcd - prav� horn� roh
extern void writeTime();

// zobraz� na lcd statistiky komunikace
extern void commStat();

// zobraz� na LCD stav joysticku
extern void joy();

// stand. re�im displeje
extern void standBy();

// odesl�n� paketu - extra funkce pro ka�d� modul
extern void sendPacketE();

// prov��� komunikaci s modulem zadan� adresy
// vrac� �sp�nost v %
extern uint8_t sendEcho(uint8_t addr);

extern void initModule(char *mod_name, uint8_t addr);

// inicializace modul� - echo
extern void initModules();

// odesl�n� statistiky komunikace s moduly do PC
extern void sendCommStat();





#endif
