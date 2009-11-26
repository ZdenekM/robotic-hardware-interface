// SensMod - kód modulu obsluhujícího senzory
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface


// TODO: obsluha kompasu v přerušení

#define F_CPU 16000000UL

#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "../CommLib/comm.h"
#include "SensMod.h"


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

#define LED PORTD, 3
#define US_TRIG PORTB, 2
#define US_ECHO PORTB, 0
#define RS485_SEND PORTD, 2

// TODO: uložení adresy do EEPROM
// adresa na RS485
#define ADDR 21


// GLOBÁLNÍ PROMĚNNÉ
// stav komunikace
static tcomm_state comm_state;

// stav senzorů
static tmod_state mod_state;


void state_init(tmod_state *m) {

	m->s_state = S_FAST_SCAN;

	for (uint8_t i=0; i < 4; i++)
	m->sharp[i] = 0;

	m->us_fast = 0;

	for (uint8_t i=0; i < 5; i++)
	m->us_full[i] = 0;

	m->comp = 0;

	m->tact = 0;

	m->full_flag = 0;

	m->us_full_idx = 0;


}

uint8_t i2c_confWait() {

	mod_state.comp_to = 0;
	while(!(TWCR & 0x80) && (mod_state.comp_to<2));

	uint8_t pom = mod_state.comp_to;

	mod_state.comp_to = 0;

	return (pom<2);

}

// přečtení registru z i2c zařízení
uint8_t i2c_read(uint8_t address, uint8_t reg)
{

	uint8_t read_data = 0;

	TWCR = 0xA4;										// send a start bit on i2c bus
	if (i2c_confWait()) {	// wait for confirmation of transmit


	   TWDR = address;                                               // load address of i2c device
	   TWCR = 0x84;                                                  // transmit
	   if (!i2c_confWait()) return 0;                                        // wait for confirmation of transmit

	   TWDR = reg;                                                   // send register number to read from
	   TWCR = 0x84;                                                  // transmit
	   if (!i2c_confWait()) return 0;                                         // wait for confirmation of transmit

	   TWCR = 0xA4;                                                  // send repeated start bit
	   if (!i2c_confWait()) return 0;                                         // wait for confirmation of transmit

	   TWDR = address+1;                                             // transmit address of i2c device with readbit set
	   TWCR = 0xC4;                                                  // clear transmit interupt flag
	   if (!i2c_confWait()) return 0;                                         // wait for confirmation of transmit

	   TWCR = 0x84;                                                  // transmit, nack (last byte request)
	   if (!i2c_confWait()) return 0;                                         // wait for confirmation of transmit

	   read_data = TWDR;                                             // and grab the target data
	   TWCR = 0x94;                                                  // send a stop bit on i2c bus
	   return read_data;

	} else return 0;

}

// inicializace io
static inline void ioinit (void) {

	DDRD = (1<<DDD1)|(1<<DDD2)|(1<<DDD3);
	PORTD = (1<<PORTD3);

	DDRB = (1<<DDB2)|(1<<DDB3);
	PORTB = (1<<PORTB3);


	C_CLEARBIT(LED);


	// řízení serva
	TCCR2 = (1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS21)|(1<<CS20)|(1<<COM21)|(0<<COM20); // fast pwm, 1024x, non-inverting mode
	OCR2 = 22; // 22 - střední poloha serva, 35 - zcela vlevo, 9 - zcela vpravo, 15 - vpravo (45st), 28 - vlevo (45st)


	// RS485 9n2
	//UBRRL = 103; // 9600
	//UBRRL = 68; //14400
	//UBRRL = 25; // 38400
	//UBRRL = 12; // 76800
	UBRRL = 12;
	UBRRH = 0;

	UCSRA = (0<<U2X)|(1<<MPCM);
	UCSRB = (1<<UCSZ2)|(1<<UDRE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// ct0 - pro obecné použití
	TCCR0 = (1<<CS02)|(0<<CS01)|(0<<CS00);

	// reference = internal, 2.56V
	ADMUX = (1<<REFS1)|(1<<REFS0);

	// povolení ADC, free running mode
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(0<<ADFR)|(0<<ADSC)|(0<<ADIE);

	// nastavení přerušení od časovačů
	TIMSK = (1<<TICIE1)|(1<<TOIE1)|(1<<TOIE0);

	// nastavení I2C
	TWBR = 32; // 100khz i2c bus speed

	// inicializace struktury
	comm_state_init(&comm_state);

	state_init(&mod_state);

	// povolení příjmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);

}



// input capture
ISR(TIMER1_CAPT_vect){

	// náběžná hrana
	if (CHECKBIT(TCCR1B,ICES1)) {

		// nastavení počátku
		TCNT1 = 0;
		ICR1 = 0;

		// přepnutí na detekci sestupné hrany
		CLEARBIT(TCCR1B,ICES1);

	} else {

		// TODO: dodělat kalibraci podle teploty

		// uložení výsledku - v mm
		if (mod_state.s_state == S_FAST_SCAN) {

			/*static uint16_t pICR1 = 0;

			mod_state.us_fast = (9*pICR1 + 1*ICR1)/120-10;

			pICR1 = ICR1;*/

			//mod_state.us_fast = (9*mod_state.us_fast + 1*(ICR1/12-10))/10;


			uint16_t new = ICR1/12-10;

			mod_state.us_fast = (7*mod_state.us_fast + new)/8;

			//mod_state.us_fast = ICR1/12-10;


			mod_state.s_state = S_DONE;

		}
		else mod_state.us_full[mod_state.us_full_idx] = ICR1/12-10;

		// zastavení měření
		CLEARBIT(TCCR1B,CS11);

	}

}

// přetečení CT1 -> chyba měření us
ISR(TIMER1_OVF_vect){

	// v dosahu není žádná překážka (t>30ms)
	if (mod_state.s_state==S_FAST_SCAN) {

		mod_state.us_fast = 0;
		mod_state.s_state = S_DONE;

	}
	else mod_state.us_full[mod_state.us_full_idx] = 0;



	// zastavení měření
	CLEARBIT(TCCR1B,CS11);

}


// USART - Rx Complete
ISR(USART_RXC_vect) {

	//if (CHECKBIT(UCSRA,FE)) comm_state.frame_error++;
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

uint8_t testUS() {

	return CHECKBIT(TCCR1B,CS11);

}

// spuštění UZ měření
void startUS() {

	// pokud UZ neběží
	if (!testUS()) {

		// spuštění měření
		C_SETBIT(US_TRIG);
		_delay_us(10);
		C_CLEARBIT(US_TRIG);

		// ICNC1 - noise canceler, ICES1 - edge select (1=rising), presc 8x
		TCCR1B = (1<<ICNC1)|(1<<ICES1)|(0<<CS12)|(1<<CS11)|(0<<CS10);


	}


}


// spočítá vzdálenost překážky z ADC
uint16_t sharpDist(uint16_t adc) {

	// objekt je příliš daleko
	if (adc<180) return 0;

	// přepočet - zjištěno regresní analýzou
	uint16_t pom = (125000/adc)-47-25;

	return pom;

}

// 245 Hz
// přerušení pro obecné použití
ISR(TIMER0_OVF_vect){

	static uint8_t f10hz=0;

	if (++f10hz==25) {

		mod_state.f10hz_flag = 1;
		f10hz = 0;
	}

	static uint8_t sharp_idx = 0;

	if (!mod_state.comp_act)
	// obsluha čidel Sharp
	switch (sharp_idx) {

	case 0: {

		// sharp 1 ještě nebyl nastaven
		if ((ADMUX & 0x0F) != 6) {

			ADMUX &= 0xF0;
			ADMUX |= 6;
			SETBIT(ADCSRA,ADSC);

		} else

		// měření dokončeno
		if (!CHECKBIT(ADCSRA,ADSC)) {

			mod_state.sharp[0] = sharpDist(ADCW);
			sharp_idx++;

		}


	} break;

	case 1: {

		// sharp 2 ještě nebyl nastaven
		if ((ADMUX&0x0F) != 7) {

			ADMUX &= 0xF0;
			ADMUX |= 7;
			SETBIT(ADCSRA,ADSC);

		} else

		if (!CHECKBIT(ADCSRA,ADSC)) {

			mod_state.sharp[1] = sharpDist(ADCW);
			sharp_idx++;

		}


		} break;

	case 2: {

		// sharp 3 ještě nebyl nastaven
		if ((ADMUX&0x0F) != 0) {

			ADMUX &= 0xF0;
			ADMUX |= 0;
			SETBIT(ADCSRA,ADSC);

		} else

		if (!CHECKBIT(ADCSRA,ADSC)) {

			mod_state.sharp[2] = sharpDist(ADCW);
			sharp_idx++;

		}


		} break;

	case 3: {

		// sharp 4 ještě nebyl nastaven
		if ((ADMUX&0x0F) != 1) {

			ADMUX &= 0xF0;
			ADMUX |= 1;
			SETBIT(ADCSRA,ADSC);

		} else

		if (!CHECKBIT(ADCSRA,ADSC)) {

			mod_state.sharp[3] = sharpDist(ADCW);
			sharp_idx = 0;

		}


		} break;


	} // switch


	// počítadlo timeoutu pro příjem po RS485
	receiveTimeout(&comm_state);

	// průběžné měření ultrazvukem
	 if (mod_state.s_state==S_FAST_SCAN && !testUS()) startUS();

	 // plné měření
	 else if (mod_state.s_state==S_FULL_SCAN) {

		enum {A0,A45,A90,A135,A180};

		static uint8_t state = A0;

		#define MS150 40 // 37 = 150ms

		static uint8_t zp=0;

		switch(state) {

		case A0: {

			if (OCR2!=35) {

				zp = MS150*2;
				OCR2 = 35;

			}

			// měření skončilo -> UZ už neběží
			if (zp==0 && !testUS()) {

				state = A45;
				mod_state.us_full_idx = 1;
				break;

			}

			// zpoždění, než se otočí servo
			if (zp==1 && !testUS()) {

				startUS();
				zp--;

			} else if (zp!=0) zp--;


		} break;

		case A45: {


			if (OCR2!=28) {

				zp = MS150;
				OCR2 = 28;

			}

			// měření skončilo
			if (zp==0 && !testUS()) {

				mod_state.us_full_idx = 2;
				state = A90;

			}

			// zpoždění, než se otočí servo
			if (zp==1 && !testUS()) {

				startUS();
				zp--;

			} else if (zp!=0) zp--;


		} break;

		// přímý směr
		case A90: {

			if (OCR2!=22) {

				zp = 2*MS150;
				OCR2 = 22;
				}

			// měření skončilo
			if (zp==0 && !testUS()) {

				mod_state.us_full_idx = 3;
				state = A135;

			}

			// zpoždění, než se otočí servo
			if (zp==1 && !testUS()) {

				startUS();
				zp--;

			} else if (zp!=0) zp--;


		} break;

		case A135: {

			if (OCR2!=15) {

				zp = MS150;
				OCR2 = 15;

			}

			// měření skončilo
			if (zp==0 && !testUS()) {

				mod_state.us_full_idx = 4;
				state = A180;

			}

			// zpoždění, než se otočí servo
			if (zp==1 && !testUS()) {

				startUS();
				zp--;

			} else if (zp!=0) zp--;



		} break;

		case A180: {


			if (OCR2!=9) {

				zp = 2*MS150;
				OCR2 = 9;
			}

			// měření skončilo
			if (zp==0 && !testUS()) {

				OCR2 = 22;
				state = A0;
				mod_state.us_full_idx = 0;
				mod_state.full_flag = 1; // příznak dokončení plného měření
				mod_state.s_state = S_DONE;

			} // if

			// zpoždění, než se otočí servo
			if (zp==1 && !testUS()) {

				startUS();
				zp--;

			} else if (zp!=0) zp--;


		} break;



		} // switch



	}


	if (mod_state.comp_to<255) mod_state.comp_to++;






}



// přečtení úhlu z kompasu
void getAngle(tmod_state *m) {

	// počká se na doměření ADC
	while(!CHECKBIT(ADCSRA,ADSC));

	// při nastaveném příznaku neprobíhá další měření
	mod_state.comp_act = 1;

	uint16_t angle=0;

	angle = i2c_read(0xC0,2) <<8; // read cmps03 angle, high byte
	angle += i2c_read(0xC0,3);

	m->comp = angle;

	mod_state.comp_act = 0;
}



int main(void) {

	ioinit();

	sei();

	while(1) {

		// přepnutí na příjem
		C_CLEARBIT(RS485_SEND);

		// měření ultrazvukem (v přerušení)
		if (mod_state.s_state == S_DONE) mod_state.s_state = S_FAST_SCAN;

		if (mod_state.f10hz_flag==1) {

			// obsluha kompasu
			getAngle(&mod_state);

			mod_state.f10hz_flag = 0;

		}

		//comm_state.receive_state = PR_WAITING;

		// čekání na příjem paketu
		//while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);


		// pokud byl přijat paket -> rozhodnutí podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

			// indikace příjmu paketu
			C_FLIPBIT(LED);

			switch(comm_state.ip.packet_type) {

			// požadavek na odeslání echo paketu
			case P_ECHO: {

				// vytvoření ECHO paketu
				makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

				// odeslání paketu
				sendPacketE();

			} break;

			// rychlé měření - jízda rovně
			case P_SENS_FAST: {

				uint8_t arr[19];

				if (!mod_state.full_flag) {

					// ultrazvuk - rychlé měření
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
					arr[0] = mod_state.us_fast;
					arr[1] = mod_state.us_fast>>8; }

					// sharp 1 (levý přední)
					arr[2] = mod_state.sharp[0];
					arr[3] = mod_state.sharp[0]>>8;

					// sharp 2 (pravý přední)
					arr[4] = mod_state.sharp[1];
					arr[5] = mod_state.sharp[1]>>8;

					// sharp 3 (levý zadní)
					arr[6] = mod_state.sharp[2];
					arr[7] = mod_state.sharp[2]>>8;

					// sharp 4 (pravý zadní)
					arr[8] = mod_state.sharp[3];
					arr[9] = mod_state.sharp[3]>>8;

					// taktilní senzory
					arr[10] = mod_state.tact;

					// kompas
					arr[11] = mod_state.comp;
					arr[12] = mod_state.comp>>8;

					makePacket(&comm_state.op,arr,13,P_SENS_FAST,0);

				} else {

					// us - 0st
					arr[0] = mod_state.us_full[0];
					arr[1] = mod_state.us_full[0]>>8;

					// us - 45st
					arr[2] = mod_state.us_full[1];
					arr[3] = mod_state.us_full[1]>>8;

					// us - 90st
					arr[4] = mod_state.us_full[2];
					arr[5] = mod_state.us_full[2]>>8;

					// us - 135st
					arr[6] = mod_state.us_full[3];
					arr[7] = mod_state.us_full[3]>>8;

					// us - 180st
					arr[8] = mod_state.us_full[4];
					arr[9] = mod_state.us_full[4]>>8;


					// sharp 1 (levý přední)
					arr[10] = mod_state.sharp[0];
					arr[11] = mod_state.sharp[0]>>8;

					// sharp 2 (pravý přední)
					arr[12] = mod_state.sharp[1];
					arr[13] = mod_state.sharp[1]>>8;

					// sharp 3 (levý zadní)
					arr[14] = mod_state.sharp[2];
					arr[15] = mod_state.sharp[2]>>8;

					// sharp 4 (pravý zadní)
					arr[16] = mod_state.sharp[3];
					arr[17] = mod_state.sharp[3]>>8;

					// taktilní senzory
					arr[18] = mod_state.tact;

					makePacket(&comm_state.op,arr,19,P_SENS_FULL,0);

					mod_state.full_flag = 0;

				}

				sendPacketE();


			} break;

			// úplné měření - na místě, s otáčením serva
			case P_SENS_FULL: {

				// počkat na dokončení měření
				while(mod_state.s_state!=S_DONE);

				// spustit plné měření
				mod_state.s_state = S_FULL_SCAN;

			} break;


			} // switch

			comm_state.receive_state = PR_WAITING;

		} // if

		else if (comm_state.receive_state == PR_TIMEOUT || comm_state.receive_state == PR_READY) comm_state.receive_state = PR_WAITING;

		//comm_state.receive_state = PR_READY;


	} // while


}
