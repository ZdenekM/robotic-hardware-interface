#ifndef SENS_H
#define SENS_H

// typ pro stav modulu
typedef enum {S_FAST_SCAN, S_FULL_SCAN, S_ERROR} ts_state;

typedef struct {

	// pole pro uložení zmìøených vzdáleností ze Sharpù
	uint16_t sharp[4];

	// stav modulu
	volatile ts_state s_state;

	// pole pro uložení full scan dat z ultrazvuku
	uint16_t us_full[5];

	// prom. pro uložení rychlého scanování (za jízdy)
	uint16_t us_fast;

	// promìnná pro uložení výsledku plného skenování (bez filtrování)
	uint16_t us_comp;

	// údaje z kompasu
	uint16_t comp;

	// taktilní senzory
	uint8_t tact;

	// timeout pro ètení kompasu
	volatile uint8_t comp_to;




} tmod_state;


#endif
