// MainMod - kód pro řídicí modul
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface
// pow.c -> obsluha PowerMod


#include "pow.h"

extern tmod_state mod_state;
extern tcomm_state comm_state;
extern tcomm_state pccomm_state;
extern tpower_state power_state;

// načte z modulu SensMod
void getPowerState() {

	// čtení stavu senzorů
	makePacket(&comm_state.op,NULL,0,P_POWSTATE,30);

	sendPacketE();

	// čekání na odpověď
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspěšné přijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

		   // nap. bat1
		   power_state.ubatt1 = comm_state.ip.data[0];

		   // nap. bat2
		   power_state.ubatt2 = comm_state.ip.data[1];

		   // celkové napětí
		   power_state.ubatt = power_state.ubatt1 + power_state.ubatt2;

		   // nap. mod
		   power_state.umod = comm_state.ip.data[2];

		   // nap. mot
		   power_state.umot = comm_state.ip.data[3];


		   comm_state.receive_state = PR_READY;

	} //if


}
