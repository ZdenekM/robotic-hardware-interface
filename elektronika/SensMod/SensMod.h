#ifndef SENS_H
#define SENS_H

// typ pro stav modulu
typedef enum {S_FAST_SCAN, S_FULL_SCAN, S_ERROR} ts_state;

typedef struct {

	// pole pro ulo�en� zm��en�ch vzd�lenost� ze Sharp�
	uint16_t sharp[4];

	// stav modulu
	volatile ts_state s_state;

	// pole pro ulo�en� full scan dat z ultrazvuku
	uint16_t us_full[5];

	// prom. pro ulo�en� rychl�ho scanov�n� (za j�zdy)
	uint16_t us_fast;

	// prom�nn� pro ulo�en� v�sledku pln�ho skenov�n� (bez filtrov�n�)
	uint16_t us_comp;

	// �daje z kompasu
	uint16_t comp;

	// taktiln� senzory
	uint8_t tact;

	// timeout pro �ten� kompasu
	volatile uint8_t comp_to;




} tmod_state;


#endif
