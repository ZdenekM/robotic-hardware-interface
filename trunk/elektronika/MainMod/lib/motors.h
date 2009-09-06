

#ifndef MOTORS_H_
#define MOTORS_H_

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

#include "../MainMod.h"

typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

// zjednodušená struktura pro ukládání stavu motorů
typedef struct {

	// žádaná rychlost v cm/s
	int16_t req_speed;

	// aktuální skutečná rychlost
	int16_t act_speed;

	// ujetá vzdálenost
	int32_t distance;

	// stav motoru
	tmotor_state state;

	// proud motorem
	uint8_t current;

	// teplota motoru
	uint8_t temp;

	// výkon motoru
	uint8_t load;

} tmotor;

enum {FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT};

typedef struct {

	tmotor m[4];

	// parametry regulatoru, * 10
	uint8_t P, I, D;

} tmotors;

// ******** MOTORY *******************************************************

void motor_init();

void motStat(tmotor *m);

void decodeMotorInfo(tmotor *mf, tmotor *mr);

void motPID();

void getMotorInfo(uint8_t addr, tmotor *front, tmotor *rear);

void setMotorSpeed(uint8_t addr, int16_t speed);

uint8_t setMotorsSpeed(int16_t left, int16_t right);

void getMotorPID(uint8_t addr);

void setMotorPID(uint8_t addr);





#endif /* MOTORS_H_ */
