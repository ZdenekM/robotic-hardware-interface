#ifndef MAIN_H
#define MAIN_H

#ifndef _AVR035_H_
#define _AVR035_H_

// from AVR035: Efficient C Coding for AVR

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))
#define FLIPBITMASK(x,y) (x ^= (y))
#define CHECKBITMASK(x,y) (x & (y))

#define VARFROMCOMB(x, y) x
#define BITFROMCOMB(x, y) y

#define C_SETBIT(comb) SETBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CLEARBIT(comb) CLEARBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_FLIPBIT(comb) FLIPBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CHECKBIT(comb) CHECKBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))

#endif

#define LCD_BL  PORTG, 1
#define BUTT4 PIND, 5
#define BUTT3 PIND, 6
#define BUTT2 PIND, 7
#define BUTT1 PING, 0

// povolen� vys�l�n� na rs485
#define RS485_SEND PORTD, 4

// p��jem / vys�l�n�
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))

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

	uint16_t comp;

} tsens;

typedef enum {R_READY,R_RUNNING} treg;

// struktura pro regul�tor zaji��uj�c� ujet� po�adovan� vzd�lenosti
typedef struct {

	// suma regul�toru
	int32_t lsum,rsum;

	// parametry regulatoru
	uint16_t P, I, D;

	// posledn� ujet� vzd�lenost
	int32_t llast_dist,rlast_dist;

	// po��te�n� vzd.
	int16_t lstart_dist,rstart_dist;

	// kolik se m� ujet
	int16_t req_dist;

	uint8_t state;

} tdist_reg;

// struktura pro regul�tor zaji��uj�c� oto�en� o zadan� �hel
typedef struct {

	// suma regul�toru
	int32_t sum;

	// parametry regulatoru
	uint16_t P, I, D;

	// posledn� hodnota
	int32_t last_angle;

	// po��te�n� azimut
	int16_t start_angle;

	// po�adovan� azimut
	int16_t req_angle;

	// stav
	uint8_t state;

} tangle_reg;

// ********** INICIALIZACE ***********************************************

// inicializace procesoru
void ioinit(void);



// ******* OSTATNI *******************************************************


// obsluha LCD
extern void manageLcd();






#endif
