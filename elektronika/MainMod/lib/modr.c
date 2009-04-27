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

// volá se z pøerušení
// ètení a zpracování stavu tlaèítek
void checkButtons (tmod_state *m) {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocné promìnné pro ošetøení zákmitù tlaèítek bez èekání
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

// volá se z pøerušení
// poèítadlo pro èas
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

// volá se z main
// vypíše èas na lcd - pravý horní roh
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

// volá se z main
// zobrazí na LCD stav joysticku
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

// stand. režim displeje
void standBy() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Standby");


}

// nastavení obou UARTù
void set_uarts() {

	// nastaveni uart1 - komunikace s moduly

	// UCSZxx = 111 -> 9bit
	// UCSZxx = 011 -> 8bit
	// RXCIEn: RX Complete Interrupt Enable
	// TXCIEn: TX Complete Interrupt Enable
	// RXENn: Receiver Enable
	// TXENn: Transmitter Enable
	// UCSZn1:0: Character Size
	// usbs - poèet stop bitù
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

	// povolení ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


}

// spustí AD pøevod pro urèení vychýlení joysticku
void update_joystick(tmod_state *m) {

	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitù
	ADMUX |= 6&0x3F; // osa Y

	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	m->joy_y = (m->joy_y + ADCW) / 2; // filtr

	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitù
	ADMUX |= 7&0x3F; // osa X

	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	m->joy_x = (m->joy_x + ADCW) / 2; // filtr



}

// regulátor pro ujetí zadané vzdálenosti
void distReg(tcomm_state *c, tdist_reg *d, tsens *s, tmotors *m) {

	if (d->req_dist!=0 && d->state==R_RUNNING) {

		int32_t le,re,lspeed,rspeed;

		int32_t lact_dist = (m->m[FRONT_LEFT].distance + m->m[REAR_LEFT].distance)/2;
		int32_t ract_dist = (m->m[FRONT_RIGHT].distance + m->m[REAR_RIGHT].distance)/2;

		// ujetá vzdálenost
		//d->moved_dist += act_dist - d->last_dist;

		le = (d->req_dist - (lact_dist - d->lstart_dist));
		re = (d->req_dist - (ract_dist - d->rstart_dist));

		// výpoèet akèního zásahu -> rychlosti
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

		// uložení pro pøíštì
		d->llast_dist = lact_dist;
		d->rlast_dist = ract_dist;

	}


}

// regulátor pro otoèení o zadaný úhel
void angleReg(tcomm_state *c, tangle_reg *a, tsens *s, tmotors *m) {

	if (a->state == R_RUNNING) {

		int16_t speed,e;

		// výpoèet odchylky (relativní, ne absolutní)
		e = a->req_angle - (s->comp - a->start_angle);

		// výpoèet  rychlosti
		speed = a->P*e;

		speed /= 100;

		// omezení rozsahu
		if (speed>250) speed = 250;
		else if (speed<-250) speed = -250;

		// hotovo (odchylka je 0.5°)
		if (labs(e)<5) {

			speed = 0;
			a->state = R_READY;

		}

		// urèení smìru otáèení
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

// nastavení regulátoru ujeté vzd.
void setDistReg(tdist_reg *d, tmotors *m, int16_t dist) {

	// nastavení poèáteèní vzdálenosti
	d->lstart_dist = (m->m[FRONT_LEFT].distance + m->m[REAR_LEFT].distance)/2;
	d->rstart_dist = (m->m[FRONT_RIGHT].distance + m->m[REAR_RIGHT].distance)/2;

	// zadání požadované vzdálenosti k ujetí
	d->req_dist = dist;

	d->state = R_RUNNING;


}


