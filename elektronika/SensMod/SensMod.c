// SensMod - k�d modulu obsluhuj�c�ho senzory
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: timeout p�i odpojen�m kompasu
// TODO: opravit pln� skenov�n� - bez �ek�n�, v p�eru�en� -> a� bude dom��eno, poslat m�sto paketu fast typ full, t�eba

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

// TODO: ulo�en� adresy do EEPROM
// adresa na RS485
#define ADDR 21


// GLOB�LN� PROM�NN�
// stav komunikace
static tcomm_state comm_state;

// stav senzor�
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


}

// TODO: opravit timeout pro kompas
uint8_t i2c_confWait() {

	mod_state.comp_to = 0;
	while((!(TWCR & 0x80)) && (mod_state.comp_to<10));

	return (mod_state.comp_to<10);

}

// p�e�ten� registru z i2c za��zen�
uint8_t i2c_read(uint8_t address, uint8_t reg)
{

	uint8_t read_data = 0;

	TWCR = 0xA4;										// send a start bit on i2c bus
	if (i2c_confWait()) {	// wait for confirmation of transmit


	   TWDR = address;                                               // load address of i2c device
	   TWCR = 0x84;                                                  // transmit
	   i2c_confWait();                                        // wait for confirmation of transmit

	   TWDR = reg;                                                   // send register number to read from
	   TWCR = 0x84;                                                  // transmit
	   i2c_confWait();                                        // wait for confirmation of transmit

	   TWCR = 0xA4;                                                  // send repeated start bit
	   i2c_confWait();                                        // wait for confirmation of transmit

	   TWDR = address+1;                                             // transmit address of i2c device with readbit set
	   TWCR = 0xC4;                                                  // clear transmit interupt flag
	   i2c_confWait();                                        // wait for confirmation of transmit

	   TWCR = 0x84;                                                  // transmit, nack (last byte request)
	   i2c_confWait();                                        // wait for confirmation of transmit

	   read_data = TWDR;                                             // and grab the target data
	   TWCR = 0x94;                                                  // send a stop bit on i2c bus
	   return read_data;

	} else return 0;

}

// vysl�n� znaku po i2c
void i2c_transmit(uint8_t address, uint8_t reg, uint8_t data)
{
   TWCR = 0xA4;                                                  // send a start bit on i2c bus
   while(!(TWCR & 0x80));                                        // wait for confirmation of transmit
   TWDR = address;                                               // load address of i2c device
   TWCR = 0x84;                                                  // transmit
   while(!(TWCR & 0x80));                                        // wait for confirmation of transmit
   TWDR = reg;
   TWCR = 0x84;                                                  // transmit
   while(!(TWCR & 0x80));                                        // wait for confirmation of transmit
   TWDR = data;
   TWCR = 0x84;                                                  // transmit
   while(!(TWCR & 0x80));                                        // wait for confirmation of transmit
   TWCR = 0x94;                                                  // stop bit
}


// inicializace io
static inline void ioinit (void) {

	DDRD = (1<<DDD1)|(1<<DDD2)|(1<<DDD3);
	PORTD = (1<<PORTD3);

	DDRB = (1<<DDB2)|(1<<DDB3);
	PORTB = (1<<PORTB3);


	C_CLEARBIT(LED);


	// ��zen� serva
	TCCR2 = (1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS21)|(1<<CS20)|(1<<COM21)|(0<<COM20); // fast pwm, 1024x, non-inverting mode
	OCR2 = 22; // 22 - st�edn� poloha serva, 35 - zcela vlevo, 9 - zcela vpravo, 15 - vpravo (45st), 28 - vlevo (45st)


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

	// ct0 - pro obecn� pou�it�
	TCCR0 = (1<<CS02)|(0<<CS01)|(1<<CS00);

	// reference = internal, 2.56V
	ADMUX = (1<<REFS1)|(1<<REFS0);

	// povolen� ADC, free running mode
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(0<<ADFR)|(0<<ADSC)|(0<<ADIE);

	// nastaven� p�eru�en� od �asova��
	TIMSK = (1<<TICIE1)|(1<<TOIE1)|(1<<TOIE0);

	// nastaven� I2C
	TWBR = 32; // 100khz i2c bus speed

	// inicializace struktury
	comm_state_init(&comm_state);

	state_init(&mod_state);

	// povolen� p��jmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);

}



// input capture
ISR(TIMER1_CAPT_vect){

	// n�b�n� hrana
	if (CHECKBIT(TCCR1B,ICES1)) {

		// nastaven� po��tku
		TCNT1 = 0;
		ICR1 = 0;

		// p�epnut� na detekci sestupn� hrany
		CLEARBIT(TCCR1B,ICES1);

	} else {

		// TODO: dod�lat kalibraci podle teploty

		// filtrov�n�
		static uint16_t pICR1;

		// ulo�en� v�sledku - v mm
		mod_state.us_fast = ((ICR1 + pICR1)/2)/12-10;
		mod_state.us_comp = ICR1/12-10;

		pICR1 = ICR1;

		// zastaven� m��en�
		CLEARBIT(TCCR1B,CS11);

	}

}

// p�ete�en� CT1 -> chyba m��en� us
ISR(TIMER1_OVF_vect){

	// v dosahu nen� ��dn� p�ek�ka (t>30ms)
	mod_state.us_fast = 0;

	// zastaven� m��en�
	CLEARBIT(TCCR1B,CS11);

}


// USART - Rx Complete
ISR(USART_RXC_vect) {

	//if (CHECKBIT(UCSRA,FE)) comm_state.frame_error++;
	receivePacket(UDR,&comm_state);

	// pr�v� p�i�la adresa
	if (comm_state.receive_state==PR_LEN) {

		// adresa sed� -> zru�it MPCM
		if (comm_state.ip.addr==ADDR) CLEARBIT(UCSRA,MPCM);

		// adresa nesed� -> nastavit p��znak
		else comm_state.receive_state = PR_READY;

	}

	// konec p��jmu v�eho
	if (comm_state.receive_state==PR_PACKET_RECEIVED || comm_state.receive_state==PR_READY ||
			comm_state.receive_state==PR_TIMEOUT) SETBIT(UCSRA,MPCM);

}


// USART - UDR Empty
ISR(USART_UDRE_vect) {

	sendPacket(&UDR,&comm_state);

	if (comm_state.send_state==PS_READY) CLEARBIT(UCSRB,UDRIE);

}

// kompletn� obsluha odesl�n� paketu
void sendPacketE() {

	// zak�z�n� p��jmu
	CLEARBIT(UCSRB,RXEN);

	// p�epnut� na odes�l�n�
	C_SETBIT(RS485_SEND);

	// odesl�n� prvn�ho bytu
	sendFirstByte(&UDR,&comm_state);

	// povolen� p�eru�en� UDRIE
	SETBIT(UCSRB,UDRIE);

	// �ek�n� na odesl�n� paketu
	while(comm_state.send_state != PS_READY);

	// �ek�n� na odesl�n� posledn�ho bytu
	while (!(UCSRA & (1<<TXC)));

	// vynulov�n� TXC - nastaven�m na jedni�ku
	SETBIT(UCSRA,TXC);

	// p�epnut� na p��jem
	C_CLEARBIT(RS485_SEND);

	// povolen� p��jmu
	SETBIT(UCSRB,RXEN);


}

// 15625 Hz
// p�eru�en� pro obecn� pou�it�
ISR(TIMER0_OVF_vect){

	// po��tadlo timeoutu pro p��jem po RS485
	receiveTimeout(&comm_state);

	// us m��en� nebylo spu�t�no
	if (!CHECKBIT(TCCR1B,CS11) && mod_state.s_state==S_FAST_SCAN) {

		// spu�t�n� m��en�
		C_SETBIT(US_TRIG);
		_delay_us(10);
		C_CLEARBIT(US_TRIG);

		// ICNC1 - noise canceler, ICES1 - edge select (1=rising), presc 8x
		TCCR1B = (1<<ICNC1)|(1<<ICES1)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	}

	static uint16_t poc = 0;

	// 50 Hz
	if (++poc==312) {

		poc = 0;

		if (mod_state.comp_to<255) mod_state.comp_to++;


	}
	// TODO: obsluha taktiln�ch senzor� -> zkop�rovat z MainMod




}

// m��en� sonarem mimo p�eru�en�
void us_measure(uint8_t index) {

	// spu�t�n� m��en�
	C_SETBIT(US_TRIG);
	_delay_us(10);
	C_CLEARBIT(US_TRIG);

	uint16_t pom;

	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	// �ek�n� na dokon�en� m��en�
	while (CHECKBIT(TCCR1B,CS11));

	// ulo�en� vzd. do pom. prom.
	pom = mod_state.us_comp;
	mod_state.us_comp = 0;

	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	// �ek�n� na dokon�en� m��en�
	while (CHECKBIT(TCCR1B,CS11));

	// ulo�en� v�sledku do pole, filtrov�n� pr�m�rem dvou m��en�
	mod_state.us_full[index] = (mod_state.us_comp + pom)/2;
	mod_state.us_comp = 0;

}

void makeFullScan() {

	//OCR2 = 22; // 22 - st�edn� poloha serva, 35 - zcela vlevo, 9 - zcela vpravo, 15 - vpravo (45st), 28 - vlevo (45st)

	mod_state.s_state = S_FULL_SCAN;

	// �ek�n� na dokon�en� p�edchoz�ho m��en�
	while (CHECKBIT(TCCR1B,CS11));

	// doba v ms pro servo na oto�en� o 45st.
	#define SDEL 150

	OCR2 = 35; // 0st
	_delay_ms(2*SDEL);
	us_measure(0);

	OCR2 = 28; // 45st
	_delay_ms(SDEL);
	us_measure(1);

	OCR2 = 22; // 90st
	_delay_ms(SDEL);
	us_measure(2);

	OCR2 = 15; // 135st
	_delay_ms(SDEL);
	us_measure(3);

	OCR2 = 9; // 180st
	_delay_ms(SDEL);
	us_measure(4);

	OCR2 = 22;
	_delay_ms(2*SDEL);

	mod_state.s_state = S_FAST_SCAN;


}


// spo��t� vzd�lenost p�ek�ky z ADC
uint16_t sharpDist(uint16_t adc) {

	// p�epo�et - zji�t�no regresn� anal�zou
	uint16_t pom = (125000/adc)-47-25;

	// nula indikuje chybov� stav - p�ekro�en� rozsahu
	if (pom < 900) return pom;
	else return 0;

}


void getChanData(uint8_t index, uint8_t chan) {

	static uint16_t sharp[4];

	ADMUX &= 0xF0;
	ADMUX |= chan;
	SETBIT(ADCSRA,ADSC);
	while (CHECKBIT(ADCSRA,ADSC));

	// pr�m�r se po��t� ze surov�ch dat, ne ze vzd�lenosti - pro v�t�� p�esnost
	sharp[index] = (9*sharp[index] + 1*ADCW)/10;
	mod_state.sharp[index] = sharpDist(sharp[index]);

}

// obsluha Sharp�
void getAnalogData() {

	getChanData(0,6);
	getChanData(1,7);
	getChanData(2,0);
	getChanData(3,1);

}

// p�e�ten� �hlu z kompasu
void getAngle(tmod_state *m) {

	uint16_t angle=0;

	angle = i2c_read(0xC0,2) <<8; // read cmps03 angle, high byte
	angle += i2c_read(0xC0,3);

	m->comp = (m->comp + angle)/2;
}



int main(void) {

	ioinit();

	sei();


	while(1) {

		// p�epnut� na p��jem
		C_CLEARBIT(RS485_SEND);

		// obsluha Sharp�
		getAnalogData();

		getAngle(&mod_state);

		comm_state.receive_state = PR_WAITING;

		// �ek�n� na p��jem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);


		// pokud byl p�ijat paket -> rozhodnut� podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

			// indikace p��jmu paketu
			C_FLIPBIT(LED);

			switch(comm_state.ip.packet_type) {

			// po�adavek na odesl�n� echo paketu
			case P_ECHO: {

				// vytvo�en� ECHO paketu
				makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

				// odesl�n� paketu
				sendPacketE();

			} break;

			// rychl� m��en� - j�zda rovn�
			case P_SENS_FAST: {

				uint8_t arr[13];

				// ultrazvuk - rychl� m��en�
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[0] = mod_state.us_fast;
				arr[1] = mod_state.us_fast>>8; }

				// sharp 1 (lev� p�edn�)
				arr[2] = mod_state.sharp[0];
				arr[3] = mod_state.sharp[0]>>8;

				// sharp 2 (prav� p�edn�)
				arr[4] = mod_state.sharp[1];
				arr[5] = mod_state.sharp[1]>>8;

				// sharp 3 (lev� zadn�)
				arr[6] = mod_state.sharp[2];
				arr[7] = mod_state.sharp[2]>>8;

				// sharp 4 (prav� zadn�)
				arr[8] = mod_state.sharp[3];
				arr[9] = mod_state.sharp[3]>>8;

				// taktiln� senzory
				arr[10] = mod_state.tact;

				// kompas
				arr[11] = mod_state.comp;
				arr[12] = mod_state.comp>>8;

				makePacket(&comm_state.op,arr,13,P_SENS_FAST,0);

				sendPacketE();

			} break;

			// �pln� m��en� - na m�st�, s ot��en�m serva
			case P_SENS_FULL: {

				// funkce realizuj�c� pln� m��en�, trv� cca 1,5 sekundy
				makeFullScan();

				uint8_t arr[19];

				// us - 0st
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[0] = mod_state.us_full[0];
				arr[1] = mod_state.us_full[0]>>8; }

				// us - 45st
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[2] = mod_state.us_full[1];
				arr[3] = mod_state.us_full[1]>>8; }

				// us - 90st
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[4] = mod_state.us_full[2];
				arr[5] = mod_state.us_full[2]>>8; }

				// us - 135st
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[6] = mod_state.us_full[3];
				arr[7] = mod_state.us_full[3]>>8; }

				// us - 180st
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[8] = mod_state.us_full[4];
				arr[9] = mod_state.us_full[4]>>8; }


				// sharp 1 (lev� p�edn�)
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[10] = mod_state.sharp[0];
				arr[11] = mod_state.sharp[0]>>8; }

				// sharp 2 (prav� p�edn�)
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[12] = mod_state.sharp[1];
				arr[13] = mod_state.sharp[1]>>8; }

				// sharp 3 (lev� zadn�)
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[14] = mod_state.sharp[2];
				arr[15] = mod_state.sharp[2]>>8; }

				// sharp 4 (prav� zadn�)
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				arr[16] = mod_state.sharp[3];
				arr[17] = mod_state.sharp[3]>>8; }

				// taktiln� senzory
				arr[18] = mod_state.tact;

				makePacket(&comm_state.op,arr,19,P_SENS_FULL,0);

				sendPacketE();



			} break;


			} // switch

		} // if

		comm_state.receive_state = PR_READY;


	} // while


}
