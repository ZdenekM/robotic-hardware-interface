// PowerMod - obsluha napájení
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: signalizace průšvihu (zkrat)
// TODO: signalizace stavu baterií

// napětí baterií (celk ADC0, 1/2 ADC1)
// napětí pro motory - za pojistkou (ADC2)
// napětí pro moduly - za pojistku (ADC3)
// nap. dělič 1:6
// U*10 = ADCW/41


#define F_CPU 11059200UL

#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "../CommLib/comm.h"
#include "PowerMod.h"


#ifndef _AVR035_H_
#define _AVR035_H_

// from AVR035: Efficient C Coding for AVR

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))
#define FLIPBITMASK(x,y) (x ^= (y))
#define CHECKBITMASK(x,y) (x & (y))

#define VARFROMCOMB(x, y) x
#define BITFROMCOMB(x, y) y

#define C_SETBIT(comb) SETBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CLEARBIT(comb) CLEARBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_FLIPBIT(comb) FLIPBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CHECKBIT(comb) CHECKBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))

#endif

// TODO: uložení adresy do EEPROM
// adresa na RS485
#define ADDR 30

#define LED4 PORTD, 5
#define LED3 PORTD, 6
#define LED2 PORTD, 7
#define LED1 PORTB, 0
#define RS485_SEND PORTD, 2
#define RELAY PORTC, 4


// GLOBÁLNÍ PROMĚNNÉ
// stav komunikace
static tcomm_state comm_state;

// proměnné modulu
static tmod_state mod_state;

// USART - Rx Complete
ISR(USART_RXC_vect) {


	//if (CHECKBIT(UCSRA,FE)) C_FLIPBIT(LED2);

	receivePacket(UDR,&comm_state);

	// právě přišla adresa
	if (comm_state.receive_state==PR_LEN) {

		// adresa sedí -> zrušit MPCM
		if (comm_state.ip.addr==ADDR) CLEARBIT(UCSRA,MPCM);

		// adresa nesedí -> nastavit příznak
		else comm_state.receive_state = PR_READY;

	}

	// konec příjmu všeho
	if (comm_state.receive_state==PR_PACKET_RECEIVED || comm_state.receive_state==PR_READY ||
			comm_state.receive_state==PR_TIMEOUT) SETBIT(UCSRA,MPCM);

}


// USART - UDR Empty
ISR(USART_UDRE_vect) {

	sendPacket(&UDR,&comm_state);

	if (comm_state.send_state==PS_READY) CLEARBIT(UCSRB,UDRIE);

}

// kompletní obsluha odeslání paketu
void sendPacketE() {

	// zakázání příjmu
	CLEARBIT(UCSRB,RXEN);

	// přepnutí na odesílání
	C_SETBIT(RS485_SEND);

	// odeslání prvního bytu
	sendFirstByte(&UDR,&comm_state);

	// povolení přerušení UDRIE
	SETBIT(UCSRB,UDRIE);

	// čekání na odeslání paketu
	while(comm_state.send_state != PS_READY);

	// čekání na odeslání posledního bytu
	while (!(UCSRA & (1<<TXC)));

	// vynulování TXC - nastavením na jedničku
	SETBIT(UCSRA,TXC);

	// přepnutí na příjem
	C_CLEARBIT(RS485_SEND);

	// povolení příjmu
	SETBIT(UCSRB,RXEN);


}

// inicializace io
static inline void ioinit (void) {

	//OSCCAL = 156;

	DDRD = (1<<DDD2)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7)|(0<<DDD0)|(1<<DDD1);
	DDRB = (1<<DDB0);
	DDRC = (1<<DDC4);

	// RS485 9n2
	UBRRL = 8;
	UBRRH = 0;

	UCSRA = (0<<U2X)|(1<<MPCM);
	UCSRB = (1<<UCSZ2)|(0<<UDRE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// reference = internal, 2.56V
	ADMUX = (1<<REFS1)|(1<<REFS0);

	// povolení ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); // /64 = 172.800 Hz

	// CT2, 1kHz
	OCR2 = 171;
	TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<CS22)|(1<<CS21)|(1<<CS20);

	TIMSK = (1<<OCIE2);

	C_CLEARBIT(LED1);
	C_CLEARBIT(LED2);
	C_CLEARBIT(LED3);
	C_CLEARBIT(LED4);

	// inicializace struktury
	comm_state_init(&comm_state);


	_delay_ms(2000);

	C_SETBIT(LED1);
	C_SETBIT(LED2);
	C_SETBIT(LED3);
	C_SETBIT(LED4);

	C_SETBIT(RELAY);

	// povolení příjmu
	C_CLEARBIT(RS485_SEND);

	sei();

}

// 1kHz přerušení pro obecné použití
ISR(TIMER2_COMP_vect) {

	static uint8_t ad_state = 0, f50hz=0, f05hz,f2hz,f4hz;

	// 50 Hz
	if (++f50hz==20) {

		// počítadlo timeoutu pro příjem po RS485
		receiveTimeout(&comm_state);

		f50hz = 0;


		if (mod_state.ubatt>=130) C_SETBIT(LED2);

		if (++f05hz==100) {

			if (mod_state.ubatt < 130 && mod_state.ubatt > 125) C_FLIPBIT(LED2);

			f05hz = 0;

		}

		if (++f2hz==25) {

			if (mod_state.ubatt <= 125 && mod_state.ubatt > 120) C_FLIPBIT(LED2);

			f2hz=0;

		}

		if (++f4hz==12) {

			if (mod_state.ubatt <= 120) C_FLIPBIT(LED2);

			f4hz=0;

				}


	}

	switch (ad_state) {

		case 0: {

			// kanál 0 ještě nebyl nastaven
			if ((ADMUX & 0x0F) != 0) {

				ADMUX &= 0xF0;
				ADMUX |= 0;
				SETBIT(ADCSRA,ADSC);

			} else

			// měření dokončeno
			if (!CHECKBIT(ADCSRA,ADSC)) {

				mod_state.ubatt = (uint8_t)((float)ADCW/6.66);
				//mod_state.ubatt1 = ADCL;
				ad_state++;

			}


		} break;

		case 1: {

			// kanál 1 ještě nebyl nastaven
			if ((ADMUX & 0x0F) != 1) {

				ADMUX &= 0xF0;
				ADMUX |= 1;
				SETBIT(ADCSRA,ADSC);

			} else

			// měření dokončeno
			if (!CHECKBIT(ADCSRA,ADSC)) {

				//mod_state.ubatt2 = ADCH;
				mod_state.ubatt2 = (uint8_t)((float)ADCW/6.66);
				mod_state.ubatt1 = mod_state.ubatt - mod_state.ubatt2;
				ad_state++;

			}


		} break;

		case 2: {

			// kanál 2 ještě nebyl nastaven
			if ((ADMUX & 0x0F) != 2) {

				ADMUX &= 0xF0;
				ADMUX |= 2;
				SETBIT(ADCSRA,ADSC);

			} else

			// měření dokončeno
			if (!CHECKBIT(ADCSRA,ADSC)) {

				mod_state.umot = (uint8_t)((float)ADCW/6.66);
				ad_state++;

			}


		} break;

		case 3: {

			// sharp 1 ještě nebyl nastaven
			if ((ADMUX & 0x0F) != 3) {

				ADMUX &= 0xF0;
				ADMUX |= 3;
				SETBIT(ADCSRA,ADSC);

			} else

			// měření dokončeno
			if (!CHECKBIT(ADCSRA,ADSC)) {

				mod_state.umod = (uint8_t)((float)ADCW/6.66);
				ad_state=0;

			}


		} break;


	} //switch

}

// vytvoření paketu
void makePowerInfo() {

	uint8_t data[4]; // 4 byty

	// napětí baterie 1
	data[0] = mod_state.ubatt1;

	// napětí baterie 2
	data[1] = mod_state.ubatt2;

	// napětí pro moduly
	data[2] = mod_state.umod;

	// napětí pro motory
	data[3] = mod_state.umot;

	makePacket(&comm_state.op,data,4,P_POWSTATE,0);

}

int main() {


	ioinit();

	sei();


	while(1) {

		// přepnutí na příjem
		C_CLEARBIT(RS485_SEND);

		comm_state.receive_state = PR_WAITING;

		// čekání na příjem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);


		// pokud byl přijat paket -> rozhodnutí podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

		// indikace příjmu paketu
		C_FLIPBIT(LED1);

			switch(comm_state.ip.packet_type) {

				// požadavek na odeslání echo paketu
				case P_ECHO: {

					// vytvoření ECHO paketu
					makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

					// odeslání paketu
					sendPacketE();

				} break;

				case P_POWSTATE: {

					// vytvoření paketu
					makePowerInfo();

					// odeslání paketu
					sendPacketE();

				} break;

			} // switch



	} // if

	comm_state.receive_state = PR_READY;

	} // while

	return 0;

}
