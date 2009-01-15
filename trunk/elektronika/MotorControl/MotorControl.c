// MotorControl - k�d pro modul ��dic� pohon
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: regul�tor
// TODO: m��en� proudu
// TODO: m��en� teploty motoru
// TODO: odes�l�n� stavov�ch dat
// TODO: plynul� rozjezd a zastaven�
// TODO: zastaven� p�i nadm�rn�m proudu motorem
/*

kolo WH100 - obvod 628,32mm
emg30 - 360 pulzu na otacku


*/


#define F_CPU 16000000UL  // 16 MHz

#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>

#include "../CommLib/comm.h"
#include "MotorControl.h"


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

#define LED PORTD, 2

#define INPUT1 PORTD, 3
#define INPUT2 PORTD, 4
#define INPUT3 PORTD, 6
#define INPUT4 PORTD, 5


#define EN1A PINC, 2
#define EN1B PINC, 3

#define EN2A PINC, 4
#define EN2B PINC, 5

#define RS485_SEND PORTB, 0

// p��jem / vys�l�n�
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))

// TODO: ulo�en� adresy do EEPROM
// adresa na RS485
#define ADDR 10


volatile uint8_t ena_st=0;

#define PEN1A ena_st, 0
#define PEN1B ena_st, 1

#define PEN2A ena_st, 2
#define PEN2B ena_st, 3

#define DEBUG ena_st, 7



// GLOB�LN� PROM�NN�
// stav komunikace
volatile tcomm_state comm_state;

volatile tmotor motor1;
volatile tmotor motor2;


// inicializace struktury motor
void motor_init(tmotor *m) {

	m->enc = 0;
	m->last_enc = 0;
	m->req_speed = 0;
	m->act_speed = 0;
	m->last_speed = 0;
	m->sum = 0;

	m->P = 4;
	m->I = 4 ;
	m->D = 4;

	m->forwd = NULL;
	m->backwd = NULL;
	m->stop = NULL;
	m->free = NULL;

	m->state = MOT_FREE;

}

// motor 2 dop�edu
void motor2_forwd(void) {

	C_SETBIT(INPUT4);
	C_CLEARBIT(INPUT3);

}

// motor 2 dozadu
void motor2_backwd(void) {

		C_SETBIT(INPUT3);
		C_CLEARBIT(INPUT4);

}

// motor 2 stop
void motor2_stop(void) {

		C_SETBIT(INPUT3);
		C_SETBIT(INPUT4);

}

// motor 2 volno
void motor2_free(void) {

		C_CLEARBIT(INPUT3);
		C_CLEARBIT(INPUT4);

}


// inicializace io
static inline void ioinit (void) {

	DDRD = (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(0<<DDD0)|(1<<DDD1);
	PORTD = (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(0<<PORTD0)|(1<<PORTD1);

	DDRB = (1<<DDB1)|(1<<DDB2)|(1<<DDB0);
	PORTB = (1<<PORTB1)|(1<<PORTB2)|(1<<PORTB0);

	DDRC = (0<<DDC2)|(0<<DDC3)|(0<<DDC4)|(0<<DDC5);
	PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5);


	C_CLEARBIT(LED);

	// RS485 9n2
	//UBRRL = 103;
	//UBRRL = 68; //14400
	//UBRRL = 25; // 38400
	//UBRRL = 12; // 76800
	UBRRL = 3; // 250000
	UBRRH = 0;

	UCSRA = (0<<U2X)|(1<<MPCM);
	UCSRB = (1<<UCSZ2)|(1<<TXCIE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// ct0, 40kHz, 8x presc, normal mode - �ten� enkod�r�
	TCCR0 = (0<<CS02)|(1<<CS01)|(0<<CS00);
	TCNT0 = 50;

	// 100 Hz - CT1 - PWM pro motory
	ICR1 = 1250; //  PWM period - TOP
	ICR1 = 1000;
	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); // phase correct, 10-bit
	TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10) | (0<<WGM13) | (0<<WGM12); // 64x presc.

	OCR1A = 0;

	OCR1B = 0;

	C_SETBIT(INPUT1);
	C_SETBIT(INPUT2);
	C_SETBIT(INPUT3);
	C_SETBIT(INPUT4);

	// inicializace struktur
	motor_init(&motor1);
	motor_init(&motor2);

	// funkce pro ovladani smeru otaceni atd.
	motor2.forwd = &motor2_forwd;
	motor2.backwd = &motor2_backwd;
	motor2.stop = &motor2_stop;
	motor2.free = &motor2_free;

	// 100 Hz -> OCR2=78, N=1024, CTC mode -> p�eru�en� pro univerz�ln� operace
	TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(1<<CS22)|(1<<CS21)|(0<<CS20);
	OCR2 = 78;
	ASSR = (0<<AS2);

	TIMSK = (1<<TOIE0)|(1<<TOIE1);

	// inicializace struktury
	comm_state_init(&comm_state);

	// povolen� p��jmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);

	sei();

}


// PID regul�tor
uint16_t motor_reg(tmotor *m) {

	// TODO: zji�t�n� aktu�ln� rychlosti, vynulovat po��tadlo tik�

		switch(m->state) {

			// motor normalne bezi - PID regulace
			case MOT_RUNNING: {

				// TODO: vypocet akcniho zasahu
				// akce = P*odchylka + I*suma + D*(speed - last_speed)

				// ur�en� sm�ru ot��en�
				//if (m->act>0) m->forwd();
				//else m->backwd();

				// omezeni max. vel. akcniho zas.
				//if (abs(m->act)>ICR1) m->act = ICR1;

				// suma = suma + odchylka

				// osetreni max hodnoty sumy
				//#define MAXSUM 5000
				//if (m->sum > MAXSUM) m->sum = MAXSUM;
				//	else if (m->sum < -MAXSUM) m->sum = -MAXSUM;

				// ulozeni aktualni rychlosti
				// m->last_speed = m->act_speed;


			} break;

			// motor stoji - brzdeni zkratem
			case MOT_STOP: {

						m->stop();
						m->act = ICR1;

			} break;

			// motor ma volno
			case MOT_FREE: {

						m->free();
						m->act = 0;

			} break;

			// aktivni brzda
			case MOT_BRAKE: {

				// regulator pro brzdu - v podstate servo

			} break;


		} // switch


	return 0;



}

// 100 Hz -> regulace
ISR(TIMER1_OVF_vect) {


	OCR1B = motor_reg(&motor2);
	//OCR1A = motor_reg(&motor1);


}


// cteni enkoderu - nejde to vyresit nejak univerzalne??
void read_enc(void) {

	// vzestupna hrana - EN2A
		if ((C_CHECKBIT(EN2A)) && (!C_CHECKBIT(PEN2A))) {
			if (C_CHECKBIT(EN2B)) motor2.enc++;
				else motor2.enc--;
		} else
		// sestupna hrana - EN2A
		if ((!C_CHECKBIT(EN2A)) && (C_CHECKBIT(PEN2A))) {
			if (!C_CHECKBIT(EN2B)) motor2.enc++;
				else motor2.enc--;
		}




		// vzestupna hrana - EN2B
		if ((C_CHECKBIT(EN2B)) && (!C_CHECKBIT(PEN2B))) {
			if (!C_CHECKBIT(EN2A)) motor2.enc++;
				else motor2.enc--;
		} else
		// sestupna hrana - EN2B
		if ((!C_CHECKBIT(EN2B)) && (C_CHECKBIT(PEN2B))) {
			if (C_CHECKBIT(EN2A)) motor2.enc++;
				else motor2.enc--;
		}


		// ulozeni minuleho stavu EN2A
		if (C_CHECKBIT(EN2A)) C_SETBIT(PEN2A);
			else C_CLEARBIT(PEN2A);

		// ulozeni minuleho stavu EN2B
		if (C_CHECKBIT(EN2B)) C_SETBIT(PEN2B);
			else C_CLEARBIT(PEN2B);



		// vzestupna hrana - EN1A
		if ((C_CHECKBIT(EN1A)) && (!C_CHECKBIT(PEN1A))) {
			if (C_CHECKBIT(EN1B)) motor1.enc++;
				else motor1.enc--;
		} else
		// sestupna hrana - EN1A
		if ((!C_CHECKBIT(EN1A)) && (C_CHECKBIT(PEN1A))) {
			if (!C_CHECKBIT(EN1B)) motor1.enc++;
				else motor1.enc--;
		}




		// vzestupna hrana - EN1B
		if ((C_CHECKBIT(EN1B)) && (!C_CHECKBIT(PEN1B))) {
			if (!C_CHECKBIT(EN1A)) motor1.enc++;
				else motor1.enc--;
		} else
		// sestupna hrana - EN1B
		if ((!C_CHECKBIT(EN1B)) && (C_CHECKBIT(PEN1B))) {
			if (C_CHECKBIT(EN1A)) motor1.enc++;
				else motor1.enc--;
		}


		// ulozeni minuleho stavu EN1A
		if (C_CHECKBIT(EN1A)) C_SETBIT(PEN1A);
			else C_CLEARBIT(PEN1A);

		// ulozeni minuleho stavu EN1B
		if (C_CHECKBIT(EN1B)) C_SETBIT(PEN1B);
			else C_CLEARBIT(PEN1B);

}


// �ten� stavu enkod�ru - 40kHz
ISR(TIMER0_OVF_vect) {

	read_enc();

}

// USART - Rx Complete
ISR(USART_RXC_vect) {

	if (CHECKBIT(UCSRA,FE)) comm_state.frame_error++;
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
			comm_state.receive_state==PR_TIMEOUT || comm_state.receive_state==PR_BAD_CRC) SETBIT(UCSRA,MPCM);

}


// USART - Tx Complete
ISR(USART_TXC_vect) {

	sendPacket(&UDR,&comm_state);

}

// kompletn� obsluha odesl�n� paketu
void sendPacketE() {

	// zak�z�n� p��jmu
	CLEARBIT(UCSRB,RXEN);

	// p�epnut� na odes�l�n�
	C_SETBIT(RS485_SEND);

	// odesl�n� prvn�ho bytu
	sendFirstByte(&UDR,&comm_state);

	// �ek�n� na odesl�n� paketu
	while(comm_state.send_state != PS_READY);

	// p�epnut� na p��jem
	C_CLEARBIT(RS485_SEND);

	// povolen� p��jmu
	SETBIT(UCSRB,RXEN);


}


int main(void) {

	ioinit();

	C_SETBIT(LED);

	while(1) {

		// p�epnut� na p��jem
		C_CLEARBIT(RS485_SEND);


		comm_state.receive_state = PR_WAITING;
		// �ek�n� na p��jem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state != PR_BAD_CRC && comm_state.receive_state != PR_READY);

		//if (comm_state.receive_state==PR_BAD_CRC)C_FLIPBIT(LED);

		// pokud byl p�ijat paket -> rozhodnut� podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED)
		switch(comm_state.ip.packet_type) {

		case P_ECHO: {

			C_FLIPBIT(LED);

			// vytvo�en� ECHO paketu
			makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

			// odesl�n� paketu
			sendPacketE();

		} break;

		} // switch

		comm_state.receive_state = PR_READY;



	}


}
