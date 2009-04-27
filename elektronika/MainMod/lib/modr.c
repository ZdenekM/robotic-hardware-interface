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

#include "modr.h"

// vol� se z p�eru�en�
// �ten� a zpracov�n� stavu tla��tek
void checkButtons (tmod_state *m) {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocn� prom�nn� pro o�et�en� z�kmit� tla��tek bez �ek�n�
	static uint8_t pbutt1=0, pbutt2=0, pbutt3=0, pbutt4=0;

	if (!C_CHECKBIT(BUTT1) && ++pbutt1==BUTT_DEBOUNCE) {
		//pbutt1 = 0;
		SETBIT(m->buttons,ABUTT1);
	}
	if (C_CHECKBIT(BUTT1)) pbutt1=0;


	if (!C_CHECKBIT(BUTT2) && ++pbutt2==BUTT_DEBOUNCE) {
		//pbutt2 = 0;
		SETBIT(m->buttons,ABUTT2);
	}
	if (C_CHECKBIT(BUTT2)) pbutt2=0;

	if (!C_CHECKBIT(BUTT3) && ++pbutt3==BUTT_DEBOUNCE) {
		//pbutt3 = 0;
		SETBIT(m->buttons,ABUTT3);
	}
	if (C_CHECKBIT(BUTT3)) pbutt3=0;

	if (!C_CHECKBIT(BUTT4) && ++pbutt4==BUTT_DEBOUNCE) {
		//pbutt4 = 0;
		SETBIT(m->buttons,ABUTT4);
	}
	if (C_CHECKBIT(BUTT4)) pbutt4=0;

}

// vol� se z p�eru�en�
// po��tadlo pro �as
void updateTime(tmod_state *m)
{


	if (m->msec+=10, m->msec==1000) {
		m->msec = 0;
		if (++m->sec==60) {
			m->sec=0;
			if (++m->min==60) {
				m->min = 0;
				if (++m->hrs==24) m->hrs=0;
			}
		}
	}


}

// vol� se z main
// vyp�e �as na lcd - prav� horn� roh
void writeTime(tmod_state *m) {

	char ctrl=' ';

	switch(m->control) {

	case C_AUTO: ctrl = 'A'; break;
	case C_JOY: ctrl = 'J'; break;
	case C_PC: ctrl = 'P'; break;


	}

	lcd_gotoxy(20-9,0);
	char buff[10];
	snprintf(buff,10,"%c%2u:%2u:%2u",ctrl,m->hrs,m->min,m->sec);
	lcd_puts(buff);

}

// vol� se z main
// zobraz� na LCD stav joysticku
void joy(tmod_state *m) {

	char abuff[11];

	lcd_gotoxy(0,0);
	lcd_puts_P("Joystick");

	// osa x
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("JX:%7d"),m->joy_x);
	lcd_puts(abuff);

	// osa y
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("JY:%7d"),m->joy_y);
	lcd_puts(abuff);


}

// stand. re�im displeje
void standBy() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Standby");


}

// nastaven� obou UART�
void set_uarts() {

	// nastaveni uart1 - komunikace s moduly

	// UCSZxx = 111 -> 9bit
	// UCSZxx = 011 -> 8bit
	// RXCIEn: RX Complete Interrupt Enable
	// TXCIEn: TX Complete Interrupt Enable
	// RXENn: Receiver Enable
	// TXENn: Transmitter Enable
	// UCSZn1:0: Character Size
	// usbs - po�et stop bit�
	// UMSELn - async / sync
	// UPMn1:0 parita
	// TXB81 - 9. bit

	// RS232, 38400, 8n1
	UBRR0L = 25;
	UBRR0H = 0;

	UCSR0A = (0<<U2X0)|(0<<MPCM0);
	UCSR0B = (0<<UCSZ02)|(0<<UDRIE0)|(1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (0<<USBS0)|(0<<UMSEL0)|(0<<UPM01)|(0<<UPM00)|(1<<UCSZ01)|(1<<UCSZ00);

	// RS485, 9n2
	// 9600bd - UBRR1L = 103;
	//UBRR1L = 68; // 14400
	//UBRR1L = 25; // 38400
	//UBRR1L = 12; // 76800
	UBRR1L = 12; // 250000
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(0<<UDRIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);


}



void set_adc(void) {

	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolen� ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


}

// spust� AD p�evod pro ur�en� vych�len� joysticku
void update_joystick(tmod_state *m) {

	// nastaven� kan�lu
	ADMUX &= 0xC0; // vynulov�n� 5 spodnich bit�
	ADMUX |= 6&0x3F; // osa Y

	// spust� p�evod
	SETBIT(ADCSRA,ADSC);

	// �ek� na dokon�en� p�evodu
	while (CHECKBIT(ADCSRA,ADSC ));

	m->joy_y = (m->joy_y + ADCW) / 2; // filtr

	// nastaven� kan�lu
	ADMUX &= 0xC0; // vynulov�n� 5 spodnich bit�
	ADMUX |= 7&0x3F; // osa X

	// spust� p�evod
	SETBIT(ADCSRA,ADSC);

	// �ek� na dokon�en� p�evodu
	while (CHECKBIT(ADCSRA,ADSC ));

	m->joy_x = (m->joy_x + ADCW) / 2; // filtr



}

// regul�tor pro ujet� zadan� vzd�lenosti
void distReg(tcomm_state *c, tdist_reg *d, tsens *s, tmotors *m) {

	if (d->req_dist!=0 && d->state==R_RUNNING) {

		int32_t le,re,lspeed,rspeed;

		int32_t lact_dist = (m->m[FRONT_LEFT].distance + m->m[REAR_LEFT].distance)/2;
		int32_t ract_dist = (m->m[FRONT_RIGHT].distance + m->m[REAR_RIGHT].distance)/2;

		// ujet� vzd�lenost
		//d->moved_dist += act_dist - d->last_dist;

		le = (d->req_dist - (lact_dist - d->lstart_dist));
		re = (d->req_dist - (ract_dist - d->rstart_dist));

		// v�po�et ak�n�ho z�sahu -> rychlosti
		lspeed = (d->P)*le + ((d->I)*d->lsum) - (d->D)*(lact_dist - d->llast_dist);
		rspeed = (d->P)*re + ((d->I)*d->rsum) - (d->D)*(ract_dist - d->rlast_dist);

		lspeed /= 100;
		rspeed /= 100;

		if (lspeed>250) lspeed = 250;
		else if (lspeed<-250) lspeed = -250;
		else d->lsum+= le;

		if (rspeed>250) rspeed = 250;
		else if (rspeed<-250) rspeed = -250;
		else d->rsum+= re;

		// hotovo :-)
		if (le < 10 && re < 10) {

			lspeed = 0;
			rspeed = 0;

			d->state = R_READY;

		}

		setMotorsSpeed(&c,&s,&m,lspeed,rspeed);

		// ulo�en� pro p��t�
		d->llast_dist = lact_dist;
		d->rlast_dist = ract_dist;

	}


}

// regul�tor pro oto�en� o zadan� �hel
void angleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m) {

	if (a->state == R_RUNNING) {

		int16_t speed,e;

		// v�po�et odchylky (relativn�, ne absolutn�)
		e = a->req_angle - (s->comp - a->start_angle);

		// v�po�et  rychlosti
		speed = a->P*e;

		speed /= 100;

		// omezen� rozsahu
		if (speed>250) speed = 250;
		else if (speed<-250) speed = -250;

		// hotovo (odchylka je 0.5�)
		if (labs(e)<5) {

			speed = 0;
			a->state = R_READY;

		}

		// ur�en� sm�ru ot��en�
		if (e>0) setMotorsSpeed(&c,&s,&m,speed,-speed);
		else setMotorsSpeed(&c,&s,&m,-speed,speed);



	}



}

void setAngleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m, int16_t angle) {

	if (angle>=-360 && angle<=360 && a->state==R_READY) {

		setMotorsSpeed(&c,&s,&m,0,0);
		a->req_angle = angle*10;
		a->start_angle = s->comp;
		a->state = R_RUNNING;

	}

}

// nastaven� regul�toru ujet� vzd.
void setDistReg(tdist_reg *d, tmotors *m, int16_t dist) {

	// nastaven� po��te�n� vzd�lenosti
	d->lstart_dist = (m->m[FRONT_LEFT].distance + m->m[REAR_LEFT].distance)/2;
	d->rstart_dist = (m->m[FRONT_RIGHT].distance + m->m[REAR_RIGHT].distance)/2;

	// zad�n� po�adovan� vzd�lenosti k ujet�
	d->req_dist = dist;

	d->state = R_RUNNING;


}


