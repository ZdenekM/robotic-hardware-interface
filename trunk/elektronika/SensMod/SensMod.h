#ifndef SENS_H
#define SENS_H

// typ pro stav modulu
typedef enum {S_DONE, S_FAST_SCAN, S_FULL_SCAN, S_ERROR} ts_state;

typedef struct {

	// pole pro uložení změřených vzdáleností ze Sharpů
	uint16_t sharp[4];

	// stav modulu
	volatile ts_state s_state;

	// pole pro uložení full scan dat z ultrazvuku
	volatile uint16_t us_full[5];

	// index pro aktuální položku
	volatile uint8_t us_full_idx;

	// prom. pro uložení rychlého scanování (za jízdy)
	volatile uint16_t us_fast;

	// minulá hodnota
	uint16_t us_fast_last;

	// údaje z kompasu
	uint16_t comp;

	// flag indikující aktivitu kompasu
	uint8_t comp_act:1;

	// taktilní senzory
	uint8_t tact;

	// timeout pro čtení kompasu
	volatile uint8_t comp_to;

	// flag pro odeslání full_scan
	uint8_t full_flag:1;

	// flag indikující 100ms
	uint8_t f10hz_flag:1;




} tmod_state;


#endif
