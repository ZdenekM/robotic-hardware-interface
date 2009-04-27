#define F_CPU 16000000UL

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <avr/wdt.h>

#include "sens.h"

// zobrazí informace ze senzorù na LCD
void sensInfo(tsens *s) {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),s->us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),s->us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("S1:%7u"),s->sharp[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("S2:%7u"),s->sharp[1]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("S3:%7u"),s->sharp[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("S4:%7u"),s->sharp[3]);
	lcd_puts(abuff);

	lcd_gotoxy(10,3);
	sprintf_P(abuff,PSTR("CO:%7u"),s->comp);
	lcd_puts(abuff);


}

// zobrazí informace ze senzorù na LCD
void sensFullInfo(tsens *s) {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("U1:%7u"),s->us_full[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("U2:%7u"),s->us_full[1]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("U3:%7u"),s->us_full[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("U4:%7u"),s->us_full[3]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("U5:%7u"),s->us_full[4]);
	lcd_puts(abuff);


}

// naète z modulu SensMod
void getFastSensorState(tcomm_state *c, tsens *s) {

	// ètení stavu levých motorù
	makePacket(&c->op,NULL,0,P_SENS_FAST,21);

	sendPacketE(&c);

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	c->receive_state = PR_WAITING;
	while(c->receive_state != PR_PACKET_RECEIVED && c->receive_state!=PR_TIMEOUT && c->receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (c->receive_state==PR_PACKET_RECEIVED && checkPacket(&c)) {

	   // data ze sonaru
	   s->us_fast = c->ip.data[0];
	   s->us_fast |= c->ip.data[1]<<8;

	   // levý pøední sharp
	   s->sharp[0] = c->ip.data[2];
	   s->sharp[0] |= c->ip.data[3]<<8;

	   // pravý pøední sharp
	   s->sharp[1] = c->ip.data[4];
	   s->sharp[1] |= c->ip.data[5]<<8;

	   // levý zadní sharp
	   s->sharp[2] = c->ip.data[6];
	   s->sharp[2] |= c->ip.data[7]<<8;

	   // pravý zadní sharp
	   s->sharp[3] = c->ip.data[8];
	   s->sharp[3] |= c->ip.data[9]<<8;

	   // taktilní senzory
	   s->tact = c->ip.data[10];

	   s->comp = c->ip.data[11];
	   s->comp |= c->ip.data[12]<<8;


	   };

	 c->receive_state = PR_READY;


}

// provede plné skenování a naète data ze SensMod
void getFullSensorState(tcomm_state *c, tsens *s) {

	// ètení stavu levých motorù
	makePacket(&c->op,NULL,0,P_SENS_FULL,21);

	sendPacketE(&c);

	C_CLEARBIT(RS485_SEND);

	// TODO: vyzkoušet, jestli je 1500ms dost
	// èekání na dokonèení mìøení
	//_delay_ms(1500);

	// èekání na odpovìï
	c->receive_state = PR_WAITING;
	while(c->receive_state != PR_PACKET_RECEIVED && c->receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (c->receive_state==PR_PACKET_RECEIVED && checkPacket(&c)) {

	   // data ze sonaru
	   // 0st
	   s->us_full[0] = c->ip.data[0];
	   s->us_full[0] |= c->ip.data[1]<<8;

	   // 45st
	   s->us_full[1] = c->ip.data[2];
	   s->us_full[1] |= c->ip.data[3]<<8;

	   // 90st
	   s->us_full[2] = c->ip.data[4];
	   s->us_full[2] |= c->ip.data[5]<<8;

	   // 135st
	   s->us_full[3] = c->ip.data[6];
	   s->us_full[3] |= c->ip.data[7]<<8;

	   // 180st
	   s->us_full[4] = c->ip.data[8];
	   s->us_full[4] |= c->ip.data[9]<<8;

	   // levý pøední sharp
	   s->sharp[0] = c->ip.data[10];
	   s->sharp[0] |= c->ip.data[11]<<8;

	   // pravý pøední sharp
	   s->sharp[1] = c->ip.data[12];
	   s->sharp[1] |= c->ip.data[13]<<8;

	   // levý zadní sharp
	   s->sharp[2] = c->ip.data[14];
	   s->sharp[2] |= c->ip.data[15]<<8;

	   // pravý zadní sharp
	   s->sharp[3] = c->ip.data[16];
	   s->sharp[3] |= c->ip.data[17]<<8;

	   // taktilní senzory
	   s->tact = c->ip.data[18];


	   };

	 c->receive_state = PR_READY;


}
