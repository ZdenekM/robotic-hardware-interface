#ifndef MAIN_H
#define MAIN_H

#include "../CommLib/comm.h"

// maxim�ln� rychlost (mm/s)
#define V_MAX 250

// d�lka jednoho �tvrt pulzu
#define TICK_LEN 0.0300917

// prumer kola 115 mm
// obvod kola 361,1 mm
// max rychlost (pri 150 RPM) 900 mm/s
// 1 ot kola == 12.000 imp
// 1 imp. == 0,0300917 mm
// 1 mm == 33,23 imp
// 150 RPM == 600imp za 1/50s



typedef enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP} tmotor_state;

typedef struct {

	// ak�n� z�sah (hodnota OCR1x)
	int32_t act;

	// po�et tik� enkod�ru od minule
	int16_t enc;

	// pomocn� po��tadlo impulz� pro v�po�et ujet� vzd.
	int32_t penc;

	// ��dan� rychlost
	int16_t req_speed;

	// ��dan� rychlost - s rampou
	int16_t areq_speed;

	// aktu�ln� skute�n� rychlost
	int16_t act_speed;

	// ujet� vzd�lenost (v mm)
	int32_t distance;

	// minul� skute�n� rychlost
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

	// z�t� (v %) - pr�m�r
	uint8_t load;

	// ukazatele na funkce ovladajici smer otaceni atd.
	void (*forwd)(void);
	void (*backwd)(void);
	void (*stop)(void);
	void (*free)(void);

} tmotor;

// inicializace struktury typu tmotor
static inline void motor_init(tmotor *m);

// motor 2 dop�edu
extern void motor2_forwd(void);

// motor 2 dozadu
extern void motor2_backwd(void);

// motor 2 brzda
extern void motor2_stop(void);

// motor 2 volnob�h
extern void motor2_free(void);

// inicializace modulu
static inline void ioinit (void);


// funkce implementuj�c� PID regulaci
extern uint16_t motor_reg(tmotor *m);

// �ten� enkod�r�
extern void read_enc(void);


#endif
