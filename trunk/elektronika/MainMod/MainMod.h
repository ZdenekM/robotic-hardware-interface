#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// data odes�lan� do PC
enum {COMMSTATE} tpcdata;

// stavy menu
typedef enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_MRF,M_MRR,M_PID,M_SENS,M_SENS_FULL,M_JOYSTICK} tmenu_states;

// zdroje povel� pro pod��zen� moduly - zdroj ��zen�
typedef enum {C_AUTO,C_JOY,C_PC} tcontrol;

// stav modulu - menu, tla��tka, �as atd.
typedef struct {

	// aktu�ln� stav menu
	tmenu_states menu_state;

	// ur�uje zdroj povel� pro pod��zen� moduly
	tcontrol control;

	// po��tadlo pro kr�tk� zapnut� podsv�tlen� po stisku tla��tka
	uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	#define PcCommTo 10
	// po��tadlo pro timeout komunikace s PC
	volatile uint8_t pc_comm_to;

	// stav joysticku
	uint16_t joy_x, joy_y;

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
	int16_t req_speed;

	// aktu�ln� skute�n� rychlost
	int16_t act_speed;

	// ujet� vzd�lenost
	int32_t distance;

	// stav motoru
	tmotor_state state;

	// proud motorem
	uint8_t current;

	// teplota motoru
	uint8_t temp;

	// v�kon motoru
	uint8_t load;

} tmotor;

// parametry regulatoru, * 10
uint8_t mP, mI, mD;

// stav senzor�
typedef struct {

	// pole pro ulo�en� zm��en�ch vzd�lenost� ze Sharp�
	uint16_t sharp[4];

	// pole pro ulo�en� full scan dat z ultrazvuku (0, 45, 90, 135, 180)
	uint16_t us_full[5];

	// prom. pro ulo�en� rychl�ho scanov�n� (za j�zdy)
	uint16_t us_fast;

	// taktiln� senzory
	uint8_t tact;


} tsens;

// struktura pro regul�tor zaji��uj�c� ujet� po�adovan� vzd�lenosti
typedef struct {



} tdist_reg;


// ********** INICIALIZACE ***********************************************

// inicializace procesoru
extern inline void ioinit(void);

extern inline void set_uarts();

extern inline void set_adc(void);

// prov��� komunikaci s modulem zadan� adresy
// vrac� �sp�nost v %
extern uint8_t sendEcho(uint8_t addr);

// inicializace modul�, vol� fci echo pro ka�d� modul
// v�sledky zobrazuje na LCD
extern void initModules();


// ******** MOTORY *******************************************************

// inicializace struktury pro data z motor�
extern void motor_init(tmotor *m);

// na LCD zobraz� info z motoru
extern void motStat(tmotor *m);

// nastaven� po�adovan� rychlosti motor�
extern void setMotorSpeed(uint8_t addr, int16_t speed);

// na�te z modulu info o motorech a vypln� ho do struktur tmotor
extern void getMotorInfo(uint8_t addr, tmotor *front, tmotor *rear);

// dek�duje p�ijat� info z modulu MotorControl
extern void decodeMotorInfo(tmotor *mf, tmotor *mr);


// ******* SENZORY *******************************************************

// zobraz� informace ze senzor� na LCD
extern void sensInfo();

// na�te z modulu SensMod
extern void getFastSensorState();

// provede pln� skenov�n� a na�te data ze SensMod
// pln� sk. trv� cca 1,5s
extern void getFullSensorState();



// ******* OSTATNI *******************************************************

// spust� AD p�evod pro ur�en� vych�len� joysticku
extern void update_joystick(tmod_state *m);

// obsluha tla��tek - o�et�en� z�kmit�
// vol� se periodicky - z p�eru�en�
extern void checkButtons ();

// obsluha po��t�n� �asu - vol�no periodicky z p�eru�en�
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

// obsluha LCD
extern void manageLcd();

// obsluha odesl�n� paketu
extern void sendPacketE();

// odesl�n� statistiky komunikace s moduly do PC
extern void sendCommStat();





#endif
