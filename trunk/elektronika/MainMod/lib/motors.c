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

#include "motors.h"


// inicializace struktur
void motor_init(tmotors *m) {

	for (uint8_t i=0;i<4;i++) {

		m->m[i].act_speed = 0;
		m->m[i].current = 0;
		m->m[i].distance = 0;
		m->m[i].req_speed = 0;
		m->m[i].state = 0;
		m->m[i].temp = 0;
		m->m[i].load = 0;


	}



}

// volá se z main
// na LCD zobrazí info z motoru
void motStat(tmotor *m) {

	char abuff[11];

	// pož. rychlost
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("RS:%7d"),m->req_speed);
	lcd_puts(abuff);

	// akt. rychlost
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("AS:%7d"),m->act_speed);
	lcd_puts(abuff);

	// proud motorem
	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("LO:%7u"),m->load);
	lcd_puts(abuff);

	// teplota motoru
	lcd_gotoxy(10,1);
	sprintf(abuff,"TM:%7u",m->temp);
	lcd_puts(abuff);

	// proud motorem
	lcd_gotoxy(10,2);
	sprintf(abuff,"CU:%7u",m->current);
	lcd_puts(abuff);

	// ujetá vzdálenost
	lcd_gotoxy(10,3);
	sprintf_P(abuff,PSTR("DI:%7d"),m->distance);
	lcd_puts(abuff);


}

// volá se z main
// dekóduje pøijaté info z modulu MotorControl
void decodeMotorInfo(tcomm_state *c, tmotor *mf, tmotor *mr) {

		// údaje pro pøední motor ----------------------------------

		// požadovaná rychlost
		mf->req_speed = c->ip.data[0];
		mf->req_speed |= c->ip.data[1]<<8;

		// aktuální rychlost
		mf->act_speed = c->ip.data[2];
		mf->act_speed |= c->ip.data[3]<<8;

		// stav motoru
		mf->state = c->ip.data[4];

		// proud motoru
		mf->current = c->ip.data[5];

		// teplota motoru
		mf->temp = c->ip.data[6];

		// ujetá vzdálenost
		mf->distance = c->ip.data[7];
		mf->distance  |= c->ip.data[8]<<8;
		mf->distance  |= c->ip.data[9]<<16;
		mf->distance  |= c->ip.data[10]<<24;

		// výkon motoru
		mf->load = c->ip.data[11];


		// údaje pro zadní motor -------------------------------------------------

		// požadovaná rychlost
		mr->req_speed = c->ip.data[0];
		mr->req_speed |= c->ip.data[1]<<8;

		// aktuální rychlost
		mr->act_speed = c->ip.data[12];
		mr->act_speed |= c->ip.data[13]<<8;

		// stav motoru
		mr->state = c->ip.data[14];

		// proud motoru
		mr->current = c->ip.data[15];

		// teplota motoru
		mr->temp = c->ip.data[16];

		// ujetá vzdálenost
		mr->distance = c->ip.data[17];
		mr->distance  |= c->ip.data[18]<<8;
		mr->distance  |= c->ip.data[19]<<16;
		mr->distance  |= c->ip.data[20]<<24;

		// výkon motoru
		mr->load = c->ip.data[21];

}


// zobrazení PID konstant na lcd
void motPID(tmotors *m) {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("P:%7u"),m->P);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("I:%7u"),m->I);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("D:%7u"),m->D);
	lcd_puts(abuff);

}


// naète z modulu info o motorech a vyplní ho do struktur tmotor
void getMotorInfo(tcomm_state *c, uint8_t addr, tmotor *front, tmotor *rear) {

	// ètení stavu levých motorù
	makePacket(&c->op,NULL,0,P_MOTOR_INFO,addr);

	sendPacketE(&c);

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	c->receive_state = PR_WAITING;
	while(c->receive_state != PR_PACKET_RECEIVED && c->receive_state!=PR_TIMEOUT && c->receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (c->receive_state==PR_PACKET_RECEIVED && checkPacket(&c)) {

	   // dekódování pøijatých dat
	   decodeMotorInfo(&c,front,rear);

	   };

	 c->receive_state = PR_READY;


}


// nastavení požadované rychlosti motorù
void setMotorSpeed(tcomm_state *c, uint8_t addr, int16_t speed) {

	uint8_t sarr[2];

	sarr[0] = speed;
	sarr[1] = speed>>8;

	makePacket(&c->op,sarr,2,P_MOTOR_COMM,addr);

	sendPacketE(&c);


}

// nastavení rychlosti motorù -> v závislosti na vzdálenosti pøekážky
void setMotorsSpeed(tcomm_state *c, tsens *s, tmotors *m, int16_t left, int16_t right) {

	// omezení rozsahu
	if (left>250) left=250;
	if (left<-250) left=-250;

	if (right>250) right=250;
	if (right<-250) right=-250;


	// jedeme dopøedu
	if ((left > 0) && (right > 0)) {

		int16_t lowest = 500;

		if (s->us_fast!=0 && s->us_fast<lowest) lowest = s->us_fast;
		if (s->sharp[0]!=0 && s->sharp[0]<lowest) lowest = s->sharp[0];
		if (s->sharp[1]!=0 && s->sharp[1]<lowest) lowest = s->sharp[1];

		// nebezpeèná vzdálenost - nutno omezit rychlost
		if (lowest<350) {

			if (left>(lowest-100)) left = lowest-100;
			if (right>(lowest-100)) right = lowest-100;

		}

	// jedeme dozadu
	} else if (left<0 && right < 0) {

		int16_t lowest= 350;

		if (s->sharp[2]!=0 && s->sharp[2]<lowest) lowest = s->sharp[2];
		if (s->sharp[3]!=0 && s->sharp[3]<lowest) lowest = s->sharp[3];

		// nebezpeèná vzdálenost - nutno omezit rychlost
		if (lowest<350) {

			if (abs(left)>(lowest-100)) left = -(lowest-100);
			if (abs(right)>(lowest-100)) right = -(lowest-100);

				}

	} // else -> kontrola pøi otáèení??


	// zmìna rychlosti - posílá se pouze pokus se liší nová a pùvodní hodnota
	if (m->m[FRONT_LEFT].req_speed!=left) setMotorSpeed(&c,10,left);
	if (m->m[FRONT_RIGHT].req_speed!=right)	setMotorSpeed(&c,11,right);


}

// zjištìní PID konstant
void getMotorPID(tcomm_state *c, tmotors *m, uint8_t addr) {


	// ètení stavu levých motorù
	makePacket(&c->op,NULL,0,P_MOTOR_GETPID,addr);

	sendPacketE(&c);

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	c->receive_state = PR_WAITING;
	while(c->receive_state != PR_PACKET_RECEIVED && c->receive_state!=PR_TIMEOUT && c->receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (c->receive_state==PR_PACKET_RECEIVED && checkPacket(&c)) {

		m->P = c->ip.data[0];
		m->I = c->ip.data[1];
		m->D = c->ip.data[2];

	}

	c->receive_state = PR_READY;


}

// nastavení PID parametrù
void setMotorPID(tcomm_state *c, tmotors *m, uint8_t addr) {

	uint8_t sarr[3];

	sarr[0] = m->P;
	sarr[1] = m->I;
	sarr[2] = m->D;

	makePacket(&c->op,sarr,3,P_MOTOR_SETPID,addr);

	sendPacketE(&c);


}
