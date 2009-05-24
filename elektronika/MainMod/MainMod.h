#ifndef MAIN_H
#define MAIN_H


#define LCD_BL  PORTG, 1
#define BUTT4 PIND, 5
#define BUTT3 PIND, 6
#define BUTT2 PIND, 7
#define BUTT1 PING, 0

#define F_CPU 16000000UL

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <avr/wdt.h>

#include "lib/lcd.h"
#include "lib/motors.h"
#include "lib/comm.h"
#include "lib/modr.h"
#include "lib/sens.h"
#include "lib/macros.h"
#include "../CommLib/comm.h"

// povolení vysílání na rs485
#define RS485_SEND PORTD, 4

// pøíjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


// data odesílaná do PC
enum {COMMSTATE} tpcdata;

// stavy menu
typedef enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_MRF,M_MRR,M_PID,M_SENS,M_SENS_FULL,M_DISTREG,M_JOYSTICK} tmenu_states;

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

	uint16_t comp;

} tsens;

typedef enum {R_READY,R_RUNNING,R_OBST} treg;

// struktura pro regulátor zajišující ujetí poadované vzdálenosti
typedef struct {

	// parametry regulatoru
	uint16_t P;

	// poèáteèní vzd.
	int16_t lstart_dist,rstart_dist;

	// kolik se má ujet
	int16_t req_dist;

	// stav regulátoru
	uint8_t state;

	// odchylky pro reg.
	int32_t le, re;

} tdist_reg;

// struktura pro regulátor zajišující otoèení o zadanı úhel
typedef struct {

	// suma regulátoru
	int32_t sum;

	// parametry regulatoru
	uint16_t P, I, D;

	// poslední hodnota
	int32_t last_angle;

	// poèáteèní azimut
	int16_t start_angle;

	// poadovanı azimut
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
