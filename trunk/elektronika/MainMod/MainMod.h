#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// data odesílaná do PC
enum {COMMSTATE} tpcdata;

// stav modulu - menu, tlaèítka, èas atd.
typedef struct {

	// stavy menu
	enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_JOYSTICK} tmenu_states;

	// aktuální stav menu
	volatile uint8_t menu_state;

	// poèítadlo pro krátké zapnutí podsvìtlení po stisku tlaèítka
	volatile uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	// stav joysticku
	volatile uint16_t joy_x, joy_y;

	// stav tlaèítek - po pøeètení vynulovat
	uint8_t buttons;
	#define ABUTT1 0
	#define ABUTT2 1
	#define ABUTT3 2
	#define ABUTT4 3

} tmod_state;

typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

// zjednodušená struktura pro ukládání stavu motorù
typedef struct {

	// žádaná rychlost v cm/s
	volatile int16_t req_speed;

	// aktuální skuteèná rychlost
	volatile int16_t act_speed;

	// vzdálenost ujetá od posledního povelu
	volatile int16_t distance;

	// parametry regulatoru, * 10
	volatile uint8_t P, I, D;

	// stav motoru
	volatile tmotor_state state;

	// proud motorem
	volatile uint8_t current;

	// teplota motoru
	volatile uint8_t temp;

} tmotor;


extern inline void set_uarts();

extern inline void set_adc(void);

extern uint16_t moving_average_x(uint16_t value);

extern uint16_t moving_average_y(uint16_t value);

extern uint16_t joystick_xy(uint8_t dir);

extern inline void ioinit(void);

extern void checkButtons ();

extern void updateTime();

extern char *getTimeString();

// vypíše èas na lcd - pravý horní roh
extern void writeTime();

// zobrazí na lcd statistiky komunikace
extern void commStat();

// zobrazí na LCD stav joysticku
extern void joy();

// stand. režim displeje
extern void standBy();

// odeslání paketu - extra funkce pro každý modul
extern void sendPacketE();

// provìøí komunikaci s modulem zadané adresy
// vrací úspìšnost v %
extern uint8_t sendEcho(uint8_t addr);

extern void initModule(char *mod_name, uint8_t addr);

// inicializace modulù - echo
extern void initModules();

// odeslání statistiky komunikace s moduly do PC
extern void sendCommStat();





#endif
