// MotorControl - kód pro modul øídicí pohon
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: naèítání konstant regulátoru z EEPROM, nastavení z MainMod
// TODO: mìøení proudu
// TODO: mìøení teploty motoru
// TODO: zastavení pøi nadmìrném proudu motorem
// TODO: zastavení pøi výpadku komunikace


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
#include <util/atomic.h>

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

// pøíjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))

// TODO: uložení adresy do EEPROM
// adresa na RS485
#define ADDR 10


volatile static uint8_t ena_st=0;

#define PEN1A ena_st, 0
#define PEN1B ena_st, 1

#define PEN2A ena_st, 2
#define PEN2B ena_st, 3

#define DEBUG ena_st, 7



// GLOBÁLNÍ PROMÌNNÉ
// stav komunikace
volatile static tcomm_state comm_state;

volatile static tmotor motor1;
volatile static tmotor motor2;

// poèítadlo pro timeout komunikace -> zastavení
volatile static uint8_t comm_to = 0;


// inicializace struktury motor
static inline void motor_init(volatile tmotor *m) {

	m->enc = 0;
	m->req_speed = 0;
	m->act_speed = 0;
	m->last_speed = 0;
	m->areq_speed = 0;
	m->penc = 0;
	m->enc_count = 0;
	m->sum = 0;
	m->act = 0;

	m->P = 30;
	m->I = 8;
	m->D = 4;

	m->forwd = NULL;
	m->backwd = NULL;
	m->stop = NULL;
	m->free = NULL;

	m->state = MOT_FREE;

}

// motor 2 dopøedu
void motor2_forwd(void) {

	C_SETBIT(INPUT3);
	C_CLEARBIT(INPUT4);

}

// motor 2 dozadu
void motor2_backwd(void) {

	C_SETBIT(INPUT4);
	C_CLEARBIT(INPUT3);

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

// motor 1 dopøedu
void motor1_forwd(void) {

	C_SETBIT(INPUT1);
	C_CLEARBIT(INPUT2);

}

// motor 1 dozadu
void motor1_backwd(void) {

	C_SETBIT(INPUT2);
	C_CLEARBIT(INPUT1);

}

// motor 1 stop
void motor1_stop(void) {

	C_SETBIT(INPUT1);
	C_SETBIT(INPUT2);

}

// motor 1 volno
void motor1_free(void) {

	C_CLEARBIT(INPUT1);
	C_CLEARBIT(INPUT2);

}


// inicializace io
static inline void ioinit (void) {

	DDRD = (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(0<<DDD0)|(1<<DDD1);
	PORTD = (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(0<<PORTD0)|(1<<PORTD1);

	DDRB = (1<<DDB1)|(1<<DDB2)|(1<<DDB0);
	PORTB = (1<<PORTB1)|(1<<PORTB2)|(1<<PORTB0);

	DDRC = (1<<DDC0)|(1<<DDC1)|(0<<DDC2)|(0<<DDC3)|(0<<DDC4)|(0<<DDC5);
	PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5);


	C_CLEARBIT(LED);

	// RS485 9n2
	//UBRRL = 103; // 9600
	//UBRRL = 68; //14400
	//UBRRL = 25; // 38400
	//UBRRL = 12; // 76800
	UBRRL = 103; // 250000
	UBRRH = 0;

	UCSRA = (0<<U2X)|(1<<MPCM);
	UCSRB = (1<<UCSZ2)|(1<<TXCIE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// ct2, 40kHz - ètení enkodérù, CTC mode
	// 199 - 40kHz N=1
	// 32 - 30kHz N=8
	// 49 - 20kHz N=8
	OCR2 = 49;
	TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<CS22)|(1<<CS21)|(0<<CS20);

	// 100 Hz - CT1 - PWM pro motory, regulace
	// 50 Hz -> 2500
	// 100 Hz -> 1250
	#define PWM_TOP 2500
	ICR1 = PWM_TOP; //  PWM period - TOP

	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); // phase correct, 10-bit
	TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10) | (0<<WGM13) | (0<<WGM12); // 64x presc.

	OCR1A = 0;
	OCR1B = 0;

	// reference = internal, 2.56V
	ADMUX = (1<<REFS1)|(1<<REFS0);

	// povolení ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

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

	motor1.forwd = &motor1_forwd;
	motor1.backwd = &motor1_backwd;
	motor1.stop = &motor1_stop;
	motor1.free = &motor1_free;

	// 100 Hz -> OCR2=78, N=1024, CTC mode -> pøerušení pro univerzální operace
	//TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(1<<CS22)|(1<<CS21)|(0<<CS20);
	//OCR2 = 78;
	//ASSR = (0<<AS2);

	TIMSK = (1<<TOIE1)|(1<<OCIE2);

	// inicializace struktury
	comm_state_init(&comm_state);

	// povolení pøíjmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);

	sei();

}

// volá se z pøerušení
// PID regulátor
uint16_t motor_reg(volatile tmotor *m) {

	// výpoèet aktuální rychlosti 50/33 = 1.5
	m->act_speed = m->enc + m->enc/2;

	m->penc += m->enc;

	// urèení 1s -> výpoèet ujeté vzd;
	if (++m->enc_count>50) {

		m->enc_count = 0;
		m->distance += m->penc/33;
		m->penc = 0;

	}

	// vynulování poèítadla impulzù
	m->enc = 0;

	// rampa pro žádanou rychlost
	if (m->req_speed > m->areq_speed) m->areq_speed++;
	else if (m->req_speed < m->areq_speed) m->areq_speed--;


		switch(m->state) {

			// motor normalne bezi - PID regulace
			case MOT_RUNNING: {

				// akèní zásah, odchylka
				int16_t e = m->areq_speed - m->act_speed;

				// výpoèet akèního zásahu
				// akce = P*odchylka + I*suma - D*(speed - last_speed)
				m->act = (m->P)*e + (m->I)*(m->sum) - (m->D)*(m->act_speed - m->last_speed);

				// urèení smìru otáèení
				if (m->act>0) m->forwd();
				else if (m->act < 0) m->backwd();
				else m->free();


				// pøevedení akèního zásahu na abs. hodnotu
				m->act = labs(m->act)/10;

				// omezeni max. vel. akcniho zas.
				// výpoèet sumy (suma = suma + odchylka)
				if (m->act>=PWM_TOP) m->act = PWM_TOP;
				else m->sum += e; // jen pokud je act < MAX -> aby suma nerostla nade všechny meze


				// ulozeni aktualni rychlosti
				m->last_speed = m->act_speed;

				return (uint16_t)m->act;


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

// 50 Hz -> regulace
ISR(TIMER1_OVF_vect) {

	// volani regulátorù
	OCR1B = motor_reg(&motor2);
	OCR1A = motor_reg(&motor1);

	// poèítadlo timeoutu pro pøíjem po RS485
	receiveTimeout(&comm_state);

	// zastavení v pøípadì výpadku komunikace na 1s
	if (++comm_to==50) {

		motor1.req_speed = 0;
		motor2.req_speed = 0;

	}

}

// volá se z pøerušení
// ètení enkoderù
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


// ètení stavu enkodérù
ISR(TIMER2_COMP_vect) {

	read_enc();

}

// USART - Rx Complete
ISR(USART_RXC_vect) {

	//if (CHECKBIT(UCSRA,FE)) comm_state.frame_error++;
	receivePacket(UDR,&comm_state);

	// právì pøišla adresa
	if (comm_state.receive_state==PR_LEN) {

		// adresa sedí -> zrušit MPCM
		if (comm_state.ip.addr==ADDR) CLEARBIT(UCSRA,MPCM);

		// adresa nesedí -> nastavit pøíznak
		else comm_state.receive_state = PR_READY;

	}

	// konec pøíjmu všeho
	if (comm_state.receive_state==PR_PACKET_RECEIVED || comm_state.receive_state==PR_READY ||
			comm_state.receive_state==PR_TIMEOUT || comm_state.receive_state==PR_BAD_CRC) SETBIT(UCSRA,MPCM);

}


// USART - Tx Complete
ISR(USART_TXC_vect) {

	sendPacket(&UDR,&comm_state);

}

// volá se z main
// vytvoøení paketu s info o stavu motorù
void makeMotorInfo() {

	uint8_t data[22]; // 22 bytù


	// ********************************************
	// požadovaná rychlost - stejná pro oba motory
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[0] = motor1.req_speed;
	data[1] = motor1.req_speed>>8; }

	// aktuální rychlost pøedního motoru
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[2] = motor1.act_speed;
	data[3] = motor1.act_speed>>8; }

	// stav pøedního motoru
	data[4] = motor1.state;

	// proud pøedním motorem
	data[5] = motor1.current;

	// teplota pøedního motoru
	data[6] = motor1.temp;

	// ujetá vzdálenost
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[7] = (int32_t)motor1.distance;
	data[8] = (int32_t)motor1.distance>>8;
	data[9] = (int32_t)motor1.distance>>16;
	data[10] = (int32_t)motor1.distance>>24; }


	static uint8_t pl1=0;
	uint8_t l1;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {l1 = motor1.act/25;}

	// výkon pøedního motoru (pro ICR1==2500) - DP
	data[11] =  (l1 + pl1)/2;

	pl1 = l1;

	// ********************************************
	// aktuální rychlost zadního motoru
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[12] = motor2.act_speed;
	data[13] = motor2.act_speed>>8; }

	// stav zadního motoru
	data[14] = motor2.state;

	// proud zadním motorem
	data[15] = motor2.current;

	// teplota zadního motoru
	data[16] = motor2.temp;

	// ujetá vzdálenost
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[17] = (int32_t)motor2.distance;
	data[18] = (int32_t)motor2.distance>>8;
	data[19] = (int32_t)motor2.distance>>16;
	data[20] = (int32_t)motor2.distance>>24; }

	static uint8_t pl2=0;
	uint8_t l2;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {l2 = motor2.act/25;}

	// výkon zadního motoru
	data[21] = (l2 + pl2)/2;

	pl2 = l2;

	makePacket(&comm_state.op,data,22,P_INFO,0);

}

// kompletní obsluha odeslání paketu
void sendPacketE() {

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


}


// volá se z main
// mìøení proudu motory
void motor_current() {


	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	motor1.current = (ADCW/6);

	ADMUX++;

	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	motor2.current = (ADCW/6);

	ADMUX--;

}


int main(void) {

	ioinit();

	C_SETBIT(LED);

	motor1.state = MOT_RUNNING;
	motor2.state = MOT_RUNNING;



	while(1) {


		// mìøení proudu motory
		motor_current();


		// pøepnutí na pøíjem
		C_CLEARBIT(RS485_SEND);


		comm_state.receive_state = PR_WAITING;

		// èekání na pøíjem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state != PR_READY && comm_state.receive_state!=PR_TIMEOUT);

		// pokud byl pøijat paket -> rozhodnutí podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

			comm_to = 0;

			switch(comm_state.ip.packet_type) {

			case P_ECHO: {

				// vytvoøení ECHO paketu
				makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

				// odeslání paketu
				sendPacketE();

			} break;

			case P_INFO: {

				// vytvoøení paketu
				makeMotorInfo();

				// odeslání paketu
				sendPacketE();

			} break;

			case P_COMM: {


				int16_t sp = 0;


				sp = comm_state.ip.data[0];
				sp |= (int16_t)comm_state.ip.data[1]<<8;

					if (sp > V_MAX) sp = V_MAX;
					if (sp < -V_MAX) sp = -V_MAX;

					ATOMIC_BLOCK(ATOMIC_FORCEON) {
					motor1.req_speed = sp;
					motor2.req_speed = sp;
					}


			} break;

			} // switch


		} // if



		comm_state.receive_state = PR_READY;



	} // while


}
