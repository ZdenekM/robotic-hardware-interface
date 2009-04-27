

#ifndef MCOMM_H_
#define MCOMM_H_

#include "../MainMod.h"
#include "lcd.h"

void sendPacketE(tcomm_state *c);

uint8_t sendEcho(tcomm_state *c, uint8_t addr);

void initModules(tcomm_state *c);

void commStat(tcomm_state *p);

void sendCommStat(tcomm_state *c, tcomm_state *pc);

void sendPCPacketE(tcomm_state *pc);


#endif /* MCOMM_H_ */
