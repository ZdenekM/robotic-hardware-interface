#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// data odes�lan� do PC
enum {COMMSTATE} tpcdata;

// stavy menu
typedef enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_MRF,M_MRR,M_SENS,M_JOYSTICK} tmenu_states;

// zdroje povel� pro pod��zen� moduly - zdroj ��zen�
typedef enum {C_AUTO,C_JOY,C_PC} tcontrol;

// stav modulu - menu, tla��tka, �as atd.
typedef struct {

	// aktu�ln� stav menu
	volatile tmenu_states menu_state;

	// ur�uje zdroj povel� pro pod��zen� moduly
	volatile tcontrol control;

	// po��tadlo pro kr�tk� zapnut� podsv�tlen� po stisku tla��tka
	volatile uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	// stav joysticku
	volatile uint16_t joy_x, joy_y;

	// stav tla��tek - po p�e�ten� vynulovat
	volatile uint8_t buttons;

	#define ABUTT1 0
	#define ABUTT2 1
	#define ABUTT3 2
	#define ABUTT4 3

} tmod_state;

typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

// zjednodu�en� struktura pro ukl�d�n� stavu motor�
typedef struct {

	// ��dan� rychlost v cm/s
	volatile int16_t req_speed;

	// aktu�ln� skute�n� rychlost
	volatile int16_t act_speed;

	// ujet� vzd�lenost
	volatile int32_t distance;

	// parametry regulatoru, * 10
	volatile uint8_t P, I, D;

	// stav motoru
	volatile tmotor_state state;

	// proud motorem
	volatile uint8_t current;

	// teplota motoru
	volatile uint8_t temp;

	// v�kon motoru
	volatile uint8_t load;

} tmotor;

// stav senzor�
typedef struct {

	// pole pro ulo�en� zm��en�ch vzd�lenost� ze Sharp�
	volatile uint16_t sharp[4];

	// pole pro ulo�en� full scan dat z ultrazvuku (0, 45, 90, 135, 180)
	volatile uint16_t us_full[5];

	// prom. pro ulo�en� rychl�ho scanov�n� (za j�zdy)
	volatile uint16_t us_fast;

	// taktiln� senzory
	volatile uint8_t tact;


} tsens;


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

// inicializace modul� - echo
extern void initModules();

// odesl�n� statistiky komunikace s moduly do PC
extern void sendCommStat();





#endif
