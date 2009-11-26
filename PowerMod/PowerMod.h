#ifndef SENS_H
#define SENS_H

#include "../CommLib/comm.h"

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


} tmod_state;



#endif
