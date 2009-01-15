#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// frekvence regulace
#define F_REG 50

// prumer kola
#define WHEEL_DIAM 100

// pocet pulzu enkoderu na otacku
#define PULSES_PER_REV 360

// vypocet delky jednoho pulzu
#define PULSE_LEN ((WHEEL_DIAM*3.14*2)/360)

// prevod cm/s na pocet pulzu za iteraci regulatoru
#define CM2PS(CM) (CM*10/PULSE_LEN/F_REG)


typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

typedef struct {

	// akèní zásah (hodnota OCR1x)
	volatile uint16_t act;

	// poèet tikù enkodéru od minule
	volatile int32_t enc;

	// minulá hodnota
	volatile int32_t last_enc;

	// žádaná rychlost
	volatile int16_t req_speed;

	// aktuální skuteèná rychlost
	volatile int16_t act_speed;

	// vzdálenost ujetá od posledního povelu
	volatile int16_t distance;

	// minulá skuteèná rychlost
	volatile int16_t last_speed;

	// suma odchylky
	volatile int32_t sum;

	// parametry regulatoru, * 10
	volatile uint8_t P, I, D;

	// stav motoru
	volatile tmotor_state state;

	// proud motorem
	volatile uint8_t current;

	// teplota motoru
	volatile uint8_t temp;

	// ukazatele na funkce ovladajici smer otaceni atd.
	void (*forwd)(void);
	void (*backwd)(void);
	void (*stop)(void);
	void (*free)(void);

} tmotor;

// inicializace struktury typu tmotor
extern void motor_init(tmotor *m);

// motor 2 dopøedu
extern void motor2_forwd(void);

// motor 2 dozadu
extern void motor2_backwd(void);

// motor 2 brzda
extern void motor2_stop(void);

// motor 2 volnobìh
extern void motor2_free(void);

// inicializace modulu
static inline void ioinit (void);


// funkce implementující PID regulaci
extern uint16_t motor_reg(tmotor *m);

// ètení enkodérù
extern void read_enc(void);


#endif
