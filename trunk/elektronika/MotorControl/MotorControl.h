#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// maximální rychlost (mm/s)
#define V_MAX 250

// délka jednoho čtvrt pulzu
#define TICK_LEN 0.0300917

// prumer kola 115 mm
// obvod kola 361,1 mm

// PREVOD 1:30
// 1 ot kola = 12.000 imp
// 1 imp. = 0,0300917 mm
// 1 mm = 33,23 imp

// PREVOD 1:50
// 1 ot kola = 20.000 imp
// 1 imp. = 0.018055 mm
// 1 mm = 55,38632 imp



typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

typedef struct {

	// akční zásah (hodnota OCR1x)
	int32_t act;

	// počet tiků enkodéru od minule
	int16_t enc;

	// pomocné počítadlo impulzů pro výpočet ujeté vzd.
	int32_t penc;

	// žádaná rychlost
	int16_t req_speed;

	// žádaná rychlost - s rampou
	int16_t areq_speed;

	// aktuální skutečná rychlost
	int16_t act_speed;

	// ujetá vzdálenost (v mm)
	int32_t distance;

	// minulá skutečná rychlost
	int16_t last_speed;

	// suma odchylky
	int32_t sum;

	// parametry regulatoru
	uint8_t P, I, D;

	// stav motoru
	volatile tmotor_state state;

	// proud motorem
	uint8_t current;

	// teplota motoru
	uint8_t temp;

	// zátěž (v %) - průměr
	uint8_t load;

	// ukazatele na funkce ovladajici smer otaceni atd.
	void (*forwd)(void);
	void (*backwd)(void);
	void (*stop)(void);
	void (*free)(void);

} tmotor;

// inicializace struktury typu tmotor
static inline void motor_init(tmotor *m);

// motor 2 dopředu
extern void motor2_forwd(void);

// motor 2 dozadu
extern void motor2_backwd(void);

// motor 2 brzda
extern void motor2_stop(void);

// motor 2 volnoběh
extern void motor2_free(void);

// inicializace modulu
static inline void ioinit (void);


// funkce implementující PID regulaci
extern uint16_t motor_reg(tmotor *m);

// čtení enkodérů
extern void read_enc(void);


#endif
