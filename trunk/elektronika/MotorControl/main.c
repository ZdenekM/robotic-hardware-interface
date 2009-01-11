// MotorControl - kód pro modul øídicí pohon
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: obsluha komunikace
// TODO: mereni proudu
// TODO: mereni teploty motoru
// TODO: poskytnuti diag. dat

/*

kolo WH100 - obvod 628,32mm
emg30 - 360 pulzu na otacku


*/

// frekvence regulace
#define F_REG 50

// prumer kola
#define WHEEL_DIAM 100

// pocet pulzu enkoderu na otacku
#define PULSES_PER_REV 360

// vypocet delky jednoho pulzu
#define PULSE_LEN ((WHEEL_DIAM*3.14*2)/360)

// prevod cm/s na pocet pulzu za iteraci regulatoru
#define CM2PS(CM) (CM*10/PULSE_LEN/F_REG)



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

// pøíjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


volatile uint8_t ena_st=0;

#define PEN1A ena_st, 0
#define PEN1B ena_st, 1

#define PEN2A ena_st, 2
#define PEN2B ena_st, 3

#define DEBUG ena_st, 7


enum {MOT_RUNNING, MOT_BRAKE, MOT_STOP, MOT_FREE, MOT_OVERCURRENT, MOT_OVERTEMP};

typedef struct {

	// pocet tiku enkoderu
	volatile int32_t enc;

	// uschovani minule hodnoty enc
	volatile int32_t last_enc;

	// zadana rychlost (pocet tiku za urcity cas) - reference
	volatile int16_t req_speed;

	// aktualni rychlost
	volatile int16_t act_speed;

	// minula aktualni rychlost
	volatile int16_t last_speed;

	// regulator - integral (suma) odchylky
	volatile int32_t sum;

	// regulator - ulozeni akcniho zasahu
	volatile int16_t act;

	// hodnota klouzaveho prumeru
	volatile int16_t pe;

	// velikost pole pro ulozeni prumeru
	//volatile uint8_t e_arr_len;

	// pole pro ulozeni klouzaveho prumeru odchylky + akt index
	volatile int8_t e[5], ei;

	// parametry regulatoru, * 10
	volatile float P, I, D;

	// stav motoru
	volatile uint8_t state;

	// ukazatele na funkce ovladajici smer otaceni atd.
	void (*forwd)(void);
	void (*backwd)(void);
	void (*stop)(void);
	void (*free)(void);




} motor;


// GLOBÁLNÍ PROMÌNNÉ
// stav komunikace
volatile tcomm_state comm_state;
motor motor1;
motor motor2;



// inicializace struktury motor
void motor_init(motor *m) {

	m->enc = 0;
	m->last_enc = 0;
	m->req_speed = 0;
	m->act_speed = 0;
	m->last_speed = 0;
	m->sum = 0;
	m->act = 0;

	m->P = 4;
	m->I = 4 ;
	m->D = 4;

	m->forwd = NULL;
	m->backwd = NULL;
	m->stop = NULL;
	m->free = NULL;

	m->state = MOT_FREE;


	// ulozi do pole pro odchylku same nuly
	/*for (uint8_t i=0; i<sizeof(m->e);i++)
		m->e[i] = 0;*/

}


void motor2_forwd(void) {

	C_SETBIT(INPUT4);
	C_CLEARBIT(INPUT3);

}

void motor2_backwd(void) {

		C_SETBIT(INPUT3);
		C_CLEARBIT(INPUT4);

}

void motor2_stop(void) {

		C_SETBIT(INPUT3);
		C_SETBIT(INPUT4);

}

void motor2_free(void) {

		C_CLEARBIT(INPUT3);
		C_CLEARBIT(INPUT4);

}



//#define UART_BAUD_RATE  19200





static inline void
ioinit (void) {

	DDRD = (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(0<<DDD0)|(1<<DDD1);
	PORTD = (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(0<<PORTD0)|(1<<PORTD1);

	DDRB = (1<<DDB1)|(1<<DDB2)|(1<<DDB0);
	PORTB = (1<<PORTB1)|(1<<PORTB2)|(1<<PORTB0);

	DDRC = (0<<DDC2)|(0<<DDC3)|(0<<DDC4)|(0<<DDC5);
	PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5);


	C_CLEARBIT(LED);

	//uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );


	//uart_bdr();

	// 9600bd, 8n2
	//UBRRL = 103;
	//UBRRL = 68; //14400
	//UBRRL = 25; // 38400
	//UBRRL = 12; // 76800
	UBRRL = 3; // 250000
	UBRRH = 0;

	UCSRA = (0<<U2X)|(0<<MPCM);
	UCSRB = (0<<UCSZ2)|(1<<TXCIE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// ct0, 40kHz, 8x presc, normal mode
	//TCCR0 = (0<<CS02)|(1<<CS01)|(0<<CS00);
	//TCNT0 = 50;

	// 100 Hz
	//ICR1 = 1250; //  PWM period - TOP
	/*ICR1 = 1000;
	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); // phase correct, 10-bit
	TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10) | (0<<WGM13) | (0<<WGM12); // 64x presc.

	OCR1A = 0;

	OCR1B = 0;*/

	C_SETBIT(INPUT1);
	C_SETBIT(INPUT2);
	C_SETBIT(INPUT3);
	C_SETBIT(INPUT4);


	motor_init(&motor1);
	motor_init(&motor2);

	// funkce pro ovladani smeru otaceni atd.
	motor2.forwd = &motor2_forwd;
	motor2.backwd = &motor2_backwd;
	motor2.stop = &motor2_stop;
	motor2.free = &motor2_free;

	// 100 Hz -> OCR2=78, N=1024, CTC mode
	/*TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(1<<CS22)|(1<<CS21)|(0<<CS20);
	OCR2 = 78;
	ASSR = (0<<AS2);


	TIMSK = (1<<TOIE0)|(1<<TOIE1);*/

	comm_state_init(&comm_state);

	// povolení pøíjmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);


	sei();

}



// zdroj rutiny - konference hw list
/*int16_t update_moving_average(int16_t value)
{
       static int8_t buffer[50], i=0;
			 static int16_t acc=0;

       acc = acc - buffer[i] + value;
       buffer[i] = value;
       if (++i >= 50) i = 0;

       return acc/50;     // nebo acc/N podle potøeby
}*/


void update_moving_average(motor *m, uint8_t size)
{
		//uint8_t size = 50;

		m->pe = m->pe - m->e[m->ei] + (m->req_speed - m->act_speed);

		m->e[m->ei] = (m->req_speed - m->act_speed);

		if (++m->ei >= size) m->ei = 0;

}


uint16_t motor_reg(motor *m) {


		m->act_speed = m->enc - m->last_enc;
		m->last_enc = m->enc;


	if (m->e != NULL)
		switch(m->state) {

			// motor normalne bezi - PID regulace
			case MOT_RUNNING: {


						// klouzavy prumer odchylky - soupnuti o jedno misto v poli
						// odchylka = reference - rychlost

						//m->pe = update_moving_average(m->req_speed - m->act_speed);


						//update_moving_average(m,size);
						m->pe = (m->req_speed - m->act_speed);

						// vypocet akcniho zasahu
						// akce = P*odchylka + I*suma + D*(speed - last_speed)

						int16_t speed_diff = m->act_speed - m->last_speed;

						m->act = (int16_t)( (m->P)*(m->pe) + (m->I)*(m->sum) - (m->D)*speed_diff);



						// urceni smeru otaceni
						if (m->act>0) {
							if (m->forwd!= NULL) m->forwd();
						} else {
							if (m->backwd!= NULL) m->backwd();
						}

						// omezeni max. vel. akcniho zas.
						if (abs(m->act)>ICR1) m->act = ICR1;

						// suma = suma + odchylka
						m->sum += m->pe;

						// osetreni max hodnoty sumy
						#define MAXSUM 5000
						if (m->sum > MAXSUM) m->sum = MAXSUM;
							else if (m->sum < -MAXSUM) m->sum = -MAXSUM;

						// ulozeni aktualni rychlosti
						m->last_speed = m->act_speed;


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

	// test funkcnosti - m->act = 100;
	return abs(m->act);



}

/* 100 Hz */
ISR(TIMER1_OVF_vect) {

		static uint8_t pocit = 0, pct2 = 0;

	 //50 Hz
	if (++pocit==2) {
	OCR1B = motor_reg(&motor2);
	pocit = 0;
	}
	//OCR1A = motor_reg(&motor1);

	// 1 Hz
	if (++pct2==100) {
				C_SETBIT(DEBUG);
				pct2 = 0;
				//C_FLIPBIT(LED);
			}



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


/* Vzorkovani stavu enkoderu */
ISR(TIMER0_OVF_vect) {


	read_enc();


}

// USART - Rx Complete
ISR(USART_RXC_vect) {

	if (CHECKBIT(UCSRA,FE)) comm_state.frame_error++;
	receivePacket(UDR,&comm_state);

}


// USART - Tx Complete
ISR(USART_TXC_vect) {

	sendPacket(&UDR,&comm_state);

}


int main(void) {

	ioinit();

	C_SETBIT(LED);

	while(1) {

		// pøepnutí na pøíjem
		C_CLEARBIT(RS485_SEND);

		// èekání na pøíjem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED);


		// rozhodnutí podle typu paketu
		switch(comm_state.ip.packet_type) {

		case P_ECHO: {

			C_FLIPBIT(LED);

			// vytvoøení ECHO paketu
			makePacket(&comm_state.op,&comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

			// zakázání pøíjmu
			CLEARBIT(UCSRB,RXEN);

			// pøepnutí na odesílání
			C_SETBIT(RS485_SEND);

			// odeslání prvního bytu
			sendFirstByte(&UDR,&comm_state);

			// èekání na odeslání paketu
			while(comm_state.send_state != PS_READY);

			// pøepnutí na pøíjem
			C_CLEARBIT(RS485_SEND);

			// povolení pøíjmu
			SETBIT(UCSRB,RXEN);

			comm_state.send_state = PS_READY;


		} break;


		} // switch

		comm_state.receive_state = PR_READY;



	}


}
