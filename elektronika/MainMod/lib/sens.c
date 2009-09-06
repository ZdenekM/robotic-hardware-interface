// MainMod - kód pro řídicí modul
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface
// sens.c -> funkce zajišťující obsluhu senzorů


#include "sens.h"

extern tmod_state mod_state;
extern tcomm_state comm_state;
extern tcomm_state pccomm_state;
extern tdist_reg dist_reg;
extern tangle_reg angle_reg;
extern tmotors motors;
extern tsens sens;

// zobrazí informace ze senzorů na LCD
void sensInfo() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),sens.us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),sens.us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("S1:%7u"),sens.sharp[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("S2:%7u"),sens.sharp[1]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("S3:%7u"),sens.sharp[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("S4:%7u"),sens.sharp[3]);
	lcd_puts(abuff);

	lcd_gotoxy(10,3);
	sprintf_P(abuff,PSTR("CO:%7u"),sens.comp);
	lcd_puts(abuff);


}

// zobrazí informace ze senzorů na LCD
void sensFullInfo() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("U1:%7u"),sens.us_full[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("U2:%7u"),sens.us_full[1]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("U3:%7u"),sens.us_full[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("U4:%7u"),sens.us_full[3]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("U5:%7u"),sens.us_full[4]);
	lcd_puts(abuff);


}


// načte z modulu SensMod
uint8_t getSensorState() {

	// čtení stavu senzorů
	makePacket(&comm_state.op,NULL,0,P_SENS_FAST,21);

	sendPacketE();

	// čekání na odpověď
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspěšné přijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {


		if (comm_state.ip.packet_type==P_SENS_FAST) {

		   // data ze sonaru
		   sens.us_fast = comm_state.ip.data[0];
		   sens.us_fast |= comm_state.ip.data[1]<<8;

		   // levý přední sharp
		   sens.sharp[0] = comm_state.ip.data[2];
		   sens.sharp[0] |= comm_state.ip.data[3]<<8;

		   // pravý přední sharp
		   sens.sharp[1] = comm_state.ip.data[4];
		   sens.sharp[1] |= comm_state.ip.data[5]<<8;

		   // levý zadní sharp
		   sens.sharp[2] = comm_state.ip.data[6];
		   sens.sharp[2] |= comm_state.ip.data[7]<<8;

		   // pravý zadní sharp
		   sens.sharp[3] = comm_state.ip.data[8];
		   sens.sharp[3] |= comm_state.ip.data[9]<<8;

		   // taktilní senzory
		   sens.tact = comm_state.ip.data[10];

		   sens.comp = comm_state.ip.data[11];
		   sens.comp |= comm_state.ip.data[12]<<8;

		   comm_state.receive_state = PR_READY;

		   return FAST_SCAN;

		} else if (comm_state.ip.packet_type==P_SENS_FULL) {

			// data ze sonaru
			// 0st
			sens.us_full[0] = comm_state.ip.data[0];
			sens.us_full[0] |= comm_state.ip.data[1]<<8;

			// 45st
			sens.us_full[1] = comm_state.ip.data[2];
			sens.us_full[1] |= comm_state.ip.data[3]<<8;

			// 90st
			sens.us_full[2] = comm_state.ip.data[4];
			sens.us_full[2] |= comm_state.ip.data[5]<<8;

			// 135st
			sens.us_full[3] = comm_state.ip.data[6];
			sens.us_full[3] |= comm_state.ip.data[7]<<8;

			// 180st
			sens.us_full[4] = comm_state.ip.data[8];
			sens.us_full[4] |= comm_state.ip.data[9]<<8;

			// levý přední sharp
			sens.sharp[0] = comm_state.ip.data[10];
			sens.sharp[0] |= comm_state.ip.data[11]<<8;

			// pravý přední sharp
			sens.sharp[1] = comm_state.ip.data[12];
			sens.sharp[1] |= comm_state.ip.data[13]<<8;

			// levý zadní sharp
			sens.sharp[2] = comm_state.ip.data[14];
			sens.sharp[2] |= comm_state.ip.data[15]<<8;

			// pravý zadní sharp
			sens.sharp[3] = comm_state.ip.data[16];
			sens.sharp[3] |= comm_state.ip.data[17]<<8;

			// taktilní senzory
			sens.tact = comm_state.ip.data[18];

			comm_state.receive_state = PR_READY;

			return FULL_SCAN;


		}


	   };



}
