#ifndef MCOMM_H_
#define MCOMM_H_


#include "../MainMod.h"
#include "../../CommLib/comm.h"


void sendPacketE();

uint8_t sendEcho(uint8_t addr);

void initModules();

void commStat(tcomm_state *p);

void sendCommStat();

void sendPCPacketE();

// potvrzení přijetí paketu
void confirmRec();


#endif /* MCOMM_H_ */
