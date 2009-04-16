#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// data odesílaná do PC
enum {COMMSTATE} tpcdata;

// stavy menu
typedef enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_MRF,M_MRR,M_PID,M_SENS,M_SENS_FULL,M_JOYSTICK} tmenu_states;

// zdroje povelù pro podøízené moduly - zdroj øízení
typedef enum {C_AUTO,C_JOY,C_PC} tcontrol;

// stav modulu - menu, tlaèítka, èas atd.
typedef struct {

	// aktuální stav menu
	tmenu_states menu_state;

	// urèuje zdroj povelù pro podøízené moduly
	tcontrol control;

	// poèítadlo pro krátké zapnutí podsvìtlení po stisku tlaèítka
	uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	#define PcCommTo 10
	// poèítadlo pro timeout komunikace s PC
	volatile uint8_t pc_comm_to;

	// stav joysticku
	uint16_t joy_x, joy_y;

	// stav tlaèítek - po pøeètení vynulovat
	volatile uint8_t buttons;

	#define ABUTT1 0
	#define ABUTT2 1
	#define ABUTT3 2
	#define ABUTT4 3

} tmod_state;

typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

// zjednodušená struktura pro ukládání stavu motorù
typedef struct {

	// ádaná rychlost v cm/s
	int16_t req_speed;

	// aktuální skuteèná rychlost
	int16_t act_speed;

	// ujetá vzdálenost
	int32_t distance;

	// stav motoru
	tmotor_state state;

	// proud motorem
	uint8_t current;

	// teplota motoru
	uint8_t temp;

	// vıkon motoru
	uint8_t load;

} tmotor;

// parametry regulatoru, * 10
uint8_t mP, mI, mD;

// stav senzorù
typedef struct {

	// pole pro uloení zmìøenıch vzdáleností ze Sharpù
	uint16_t sharp[4];

	// pole pro uloení full scan dat z ultrazvuku (0, 45, 90, 135, 180)
	uint16_t us_full[5];

	// prom. pro uloení rychlého scanování (za jízdy)
	uint16_t us_fast;

	// taktilní senzory
	uint8_t tact;


} tsens;

// struktura pro regulátor zajišující ujetí poadované vzdálenosti
typedef struct {



} tdist_reg;


// ********** INICIALIZACE ***********************************************

// inicializace procesoru
extern inline void ioinit(void);

extern inline void set_uarts();

extern inline void set_adc(void);

// provìøí komunikaci s modulem zadané adresy
// vrací úspìšnost v %
extern uint8_t sendEcho(uint8_t addr);

// inicializace modulù, volá fci echo pro kadı modul
// vısledky zobrazuje na LCD
extern void initModules();


// ******** MOTORY *******************************************************

// inicializace struktury pro data z motorù
extern void motor_init(tmotor *m);

// na LCD zobrazí info z motoru
extern void motStat(tmotor *m);

// nastavení poadované rychlosti motorù
extern void setMotorSpeed(uint8_t addr, int16_t speed);

// naète z modulu info o motorech a vyplní ho do struktur tmotor
extern void getMotorInfo(uint8_t addr, tmotor *front, tmotor *rear);

// dekóduje pøijaté info z modulu MotorControl
extern void decodeMotorInfo(tmotor *mf, tmotor *mr);


// ******* SENZORY *******************************************************

// zobrazí informace ze senzorù na LCD
extern void sensInfo();

// naète z modulu SensMod
extern void getFastSensorState();

// provede plné skenování a naète data ze SensMod
// plné sk. trvá cca 1,5s
extern void getFullSensorState();



// ******* OSTATNI *******************************************************

// spustí AD pøevod pro urèení vychılení joysticku
extern void update_joystick(tmod_state *m);

// obsluha tlaèítek - ošetøení zákmitù
// volá se periodicky - z pøerušení
extern void checkButtons ();

// obsluha poèítání èasu - voláno periodicky z pøerušení
extern void updateTime();

extern char *getTimeString();

// vypíše èas na lcd - pravı horní roh
extern void writeTime();

// zobrazí na lcd statistiky komunikace
extern void commStat();

// zobrazí na LCD stav joysticku
extern void joy();

// stand. reim displeje
extern void standBy();

// obsluha LCD
extern void manageLcd();

// obsluha odeslání paketu
extern void sendPacketE();

// odeslání statistiky komunikace s moduly do PC
extern void sendCommStat();





#endif
