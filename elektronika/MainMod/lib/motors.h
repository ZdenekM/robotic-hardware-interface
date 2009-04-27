

#ifndef MOTORS_H_
#define MOTORS_H_

#include "../MainMod.h"
#include "lcd.h"
#include "comm.h"


typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

// zjednodušená struktura pro ukládání stavu motorù
typedef struct {

	// žádaná rychlost v cm/s
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

void motor_init(tmotors *m);

void motStat(tmotor *m);

void decodeMotorInfo(tcomm_state *c, tmotor *mf, tmotor *mr);

void motPID();

void getMotorInfo(tcomm_state *c, uint8_t addr, tmotor *front, tmotor *rear);

void setMotorSpeed(tcomm_state *c, uint8_t addr, int16_t speed);

void setMotorsSpeed(tcomm_state *c, tsens *s, tmotors *m, int16_t left, int16_t right);

void getMotorPID(tcomm_state *c, tmotors *m, uint8_t addr);

void setMotorPID(tcomm_state *c, tmotors *m, uint8_t addr);





#endif /* MOTORS_H_ */
