#ifndef SENS_H
#define SENS_H

// typ pro stav modulu
typedef enum {S_FAST_SCAN, S_FULL_SCAN, S_ERROR} ts_state;

typedef struct {

	// pole pro ulo�en� zm��en�ch vzd�lenost� ze Sharp�
	volatile uint16_t sharp[4];

	// stav modulu
	volatile ts_state s_state;

	// pole pro ulo�en� full scan dat z ultrazvuku
	volatile uint16_t us_full[5];

	// prom. pro ulo�en� rychl�ho scanov�n� (za j�zdy)
	volatile uint16_t us_fast;

	// �daje z kompasu
	volatile uint16_t comp;


} tmod_state;


#endif