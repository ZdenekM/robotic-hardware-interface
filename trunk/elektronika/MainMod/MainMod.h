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
#include "lib/pow.h"
#include "lib/macros.h"
#include "../CommLib/comm.h"

// povolení vysílání na rs485
#define RS485_SEND PORTD, 4

// příjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


// data odesílaná do PC
enum {COMMSTATE} tpcdata;

// stavy menu
typedef enum {M_INIT,M_STANDBY,M_COMMSTAT,M_PCCOMMSTAT,M_MLF,M_MLR,M_MRF,M_MRR,M_PID,M_SENS,M_SENS_FULL,M_DISTREG,M_ANGLEREG,M_JOYSTICK} tmenu_states;

// zdroje povelů pro podřízené moduly - zdroj řízení
typedef enum {C_AUTO,C_JOY,C_PC} tcontrol;

// stav modulu - menu, tlačítka, čas atd.
typedef struct {

	// aktuální stav menu
	tmenu_states menu_state;

	// určuje zdroj povelů pro podřízené moduly
	tcontrol control;

	// počítadlo pro krátké zapnutí podsvětlení po stisku tlačítka
	uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	#define PcCommTo 10
	// počítadlo pro timeout komunikace s PC
	volatile uint8_t pc_comm_to;

	// stav joysticku
	uint16_t joy_x, joy_y;

	// offset joysticku
	int16_t joy_x_offset,joy_y_offset;

	// stav tlačítek - po přečtení vynulovat
	volatile uint8_t buttons;

	#define ABUTT1 0
	#define ABUTT2 1
	#define ABUTT3 2
	#define ABUTT4 3

} tmod_state;


// stav senzorů
typedef struct {

	// pole pro uložení změřených vzdáleností ze Sharpů
	uint16_t sharp[4];

	// pole pro uložení full scan dat z ultrazvuku (0, 45, 90, 135, 180)
	uint16_t us_full[5];

	// příznak nového plného měření
	uint8_t new_full_flag;

	// prom. pro uložení rychlého scanování (za jízdy)
	uint16_t us_fast;

	// taktilní senzory
	uint8_t tact;

	uint16_t comp;

} tsens;

typedef struct {

	// celkové napětí baterií
	uint8_t ubatt;

	// napětí baterie 1
	uint8_t ubatt1;

	// napětí baterie 2
	uint8_t ubatt2;

	// napětí pro moduly
	uint8_t umod;

	// napětí pro motory
	uint8_t umot;

} tpower_state;

typedef enum {R_READY,R_RUNNING,R_OBST} treg;

// struktura pro regulátor zajišťující ujetí požadované vzdálenosti
typedef struct {

	// parametry regulatoru
	uint16_t P;

	// počáteční vzd.
	int16_t lstart_dist,rstart_dist;

	// kolik se má ujet
	int16_t req_dist;

	// stav regulátoru
	uint8_t state;

	// odchylky pro reg.
	int32_t le, re;

} tdist_reg;

// struktura pro regulátor zajišťující otočení o zadaný úhel
typedef struct {

	// suma regulátoru
	//int32_t sum;

	// parametry regulatoru
	uint16_t P;//, I, D;

	// regulační odchylka
	int16_t e;

	// vypočtená rychlost otáčení
	int16_t speed;

	// poslední hodnota
	int16_t last_e;

	// počáteční azimut
	int16_t start_angle;

	// požadovaný azimut
	int16_t req_angle;

	// stav
	uint8_t state;

} tangle_reg;


typedef struct {

	enum {STRAIGHT, BRAKING, WFFS,TURNING};

	// stav automatu
	uint8_t state;

	// počítadlo pro vyrušení falešných poplachů
	uint8_t obsc;

	// vyhlédnutá vzdálenost
	uint16_t dist;

	// timeout pro otáčení
	uint8_t turn_to;


} trand_ride;



// ********** INICIALIZACE ***********************************************

// inicializace procesoru
void ioinit(void);



// ******* OSTATNI *******************************************************


// obsluha LCD
extern void manageLcd();



#endif
