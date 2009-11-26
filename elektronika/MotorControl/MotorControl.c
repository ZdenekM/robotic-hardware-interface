// MotorControl - kód pro modul řídicí pohon
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: měření proudu - nefunguje
// TODO: obsluha měření proudu v přerušení
// TODO: nečekat na paket ve smyčce
// TODO: měření teploty motoru

#define F_CPU 16000000UL  // 16 MHz

#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <avr/eeprom.h>
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
#define INPUT3 PORTD, 5
#define INPUT4 PORTD, 6


#define EN1A PINC, 2
#define EN1B PINC, 3

#define EN2A PINC, 4
#define EN2B PINC, 5

#define RS485_SEND PORTB, 0

// příjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))

// TODO: uložení adresy do EEPROM
// adresa na RS485
#define ADDR 11


volatile static uint8_t ena_st=0;

#define PEN1A ena_st, 0
#define PEN1B ena_st, 1

#define PEN2A ena_st, 2
#define PEN2B ena_st, 3

#define DEBUG ena_st, 7



// GLOBÁLNÍ PROMĚNNÉ
// stav komunikace
static tcomm_state comm_state;

static tmotor motor1;
static tmotor motor2;

// počítadlo pro timeout komunikace -> zastavení
static uint8_t comm_to = 0;

// parametry PID uložené v EEPROM
uint8_t EEMEM eP=105,eI=15,eD=15;


// inicializace struktury motor
static inline void motor_init(tmotor *m) {

	m->enc = 0;
	m->req_speed = 0;
	m->act_speed = 0;
	m->last_speed = 0;
	m->areq_speed = 0;
	m->penc = 0;
	//m->enc_count = 0;
	m->sum = 0;
	m->act = 0;

	m->P = eeprom_read_byte(&eP);
	m->I = eeprom_read_byte(&eI);
	m->D = eeprom_read_byte(&eD);

	m->forwd = NULL;
	m->backwd = NULL;
	m->stop = NULL;
	m->free = NULL;

	m->state = MOT_FREE;

}

// motor 2 dopředu
void motor1_forwd(void) {

	C_SETBIT(INPUT3);
	C_CLEARBIT(INPUT4);

}

// motor 2 dozadu
void motor1_backwd(void) {

	C_SETBIT(INPUT4);
	C_CLEARBIT(INPUT3);

}

// motor 2 stop
void motor1_stop(void) {

	C_SETBIT(INPUT3);
	C_SETBIT(INPUT4);

}

// motor 2 volno
void motor1_free(void) {

	C_CLEARBIT(INPUT3);
	C_CLEARBIT(INPUT4);

}

// motor 1 dopředu
void motor2_forwd(void) {

	C_SETBIT(INPUT1);
	C_CLEARBIT(INPUT2);

}

// motor 1 dozadu
void motor2_backwd(void) {

	C_SETBIT(INPUT2);
	C_CLEARBIT(INPUT1);

}

// motor 1 stop
void motor2_stop(void) {

	C_SETBIT(INPUT1);
	C_SETBIT(INPUT2);

}

// motor 1 volno
void motor2_free(void) {

	C_CLEARBIT(INPUT1);
	C_CLEARBIT(INPUT2);

}


// inicializace io
static inline void ioinit (void) {

	DDRD = (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(0<<DDD0)|(1<<DDD1);
	PORTD = (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(0<<PORTD0)|(1<<PORTD1);

	DDRB = (1<<DDB1)|(1<<DDB2)|(1<<DDB0);
	PORTB = (1<<PORTB1)|(1<<PORTB2)|(1<<PORTB0);

	DDRC = (0<<DDC0)|(0<<DDC1)|(0<<DDC2)|(0<<DDC3)|(0<<DDC4)|(0<<DDC5);
	PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5);


	C_CLEARBIT(LED);

	// RS485 9n2
	//UBRRL = 103; // 9600
	//UBRRL = 68; //14400
	//UBRRL = 25; // 38400
	//UBRRL = 12; // 76800
	UBRRL = 12;
	UBRRH = 0;

	UCSRA = (0<<U2X)|(1<<MPCM);
	UCSRB = (1<<UCSZ2)|(0<<UDRE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(1<<UCSZ1)|(1<<UCSZ0);

	// ct2, 40kHz - čtení enkodérů, CTC mode
	// 199 - 40kHz N=1
	// 32 - 30kHz N=8
	// 49 - 20kHz N=8
	OCR2 = 49;
	TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<CS22)|(1<<CS21)|(0<<CS20);

	// 100 Hz - CT1 - PWM pro motory, regulace
	// 50 Hz -> 2500
	// 100 Hz -> 1250
	//#define PWM_TOP 2500
	//ICR1 = 2500; //  PWM period - TOP

	ICR1 = 8000; // 1 kHz

	TCCR1A = (1<<WGM11) | (0<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); // phase correct, 10-bit
	TCCR1B = (0<<CS12) | (0<<CS11) | (1<<CS10) | (1<<WGM13) | (0<<WGM12); // no presc

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

	TIMSK = (1<<TOIE1)|(1<<OCIE2);

	// inicializace struktury
	comm_state_init(&comm_state);

	// povolení příjmu
	C_CLEARBIT(RS485_SEND);

	_delay_ms(2000);
	C_SETBIT(LED);

	sei();

}

// volá se z přerušení
// PID regulátor
uint16_t motor_reg(tmotor *m) {

	// výpočet aktuální rychlosti 50/33 = 1.5
	//m->act_speed = m->enc + (m->enc/2);

	m->act_speed = (m->enc*9)/10;

	// pomocné počítadlo ujetých impulzů
	m->penc += m->enc;

	// aktualizovat ujetou vzdálenost
	if (labs(m->penc)>=55) {

		// připočítat vzdálenost
		m->distance += m->penc/55;

		// uložit zbytek po dělení
		m->penc = m->penc%55;

	}


	// vynulování počítadla impulzů
	m->enc = 0;

	// rampa pro žádanou rychlost -> nejdříve se přidává/ubírá po deseti, pak po jedné
	#define ACC 5 // zrychlení z 0 na 250 za cca 1s
	if (m->req_speed > (m->areq_speed+ACC)) m->areq_speed+=ACC;
	else if (m->req_speed > m->areq_speed) m->areq_speed++;

	if (m->req_speed < (m->areq_speed-ACC)) m->areq_speed-=ACC;
	else if (m->req_speed < m->areq_speed) m->areq_speed--;


		switch(m->state) {

			// motor normalne bezi - PID regulace
			case MOT_RUNNING: {

				// akční zásah, odchylka
				int16_t e = m->areq_speed - m->act_speed;

				// výpočet akčního zásahu
				// akce = P*odchylka + I*suma - D*(speed - last_speed)
				m->act = (m->P)*e + (m->I)*(m->sum) - (m->D)*(m->act_speed - m->last_speed);

				// určení směru otáčení
				if (m->act>0) m->forwd();
				else if (m->act < 0) m->backwd();
				else m->free();

				// převedení akčního zásahu na abs. hodnotu
				m->act = labs(m->act)/10;

				// omezeni max. vel. akcniho zas.
				// výpočet sumy (suma = suma + odchylka)
				if (m->act>8000) m->act = 8000;
				else m->sum += e; // jen pokud je act < MAX -> aby suma nerostla nade všechny meze

				// ulozeni aktualni rychlosti
				m->last_speed = m->act_speed;

				// výpočet zátěže v %
				m->load = m->act/80;

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

				// regulator pro brzdu - vpodstate servo

			} break;


		} // switch


	return 0;





}

// 1 kHz
ISR(TIMER1_OVF_vect) {

	static uint8_t p=0;

	// 50 Hz
	if (++p==20) {

		p=0;

		// volani regulátorů
		OCR1B = motor_reg(&motor2);
		OCR1A = motor_reg(&motor1);

		// počítadlo timeoutu pro příjem po RS485
		receiveTimeout(&comm_state);


		// zastavení v případě výpadku komunikace na 1s
		if (comm_to==50) {

			motor1.req_speed = 0;
			motor2.req_speed = 0;

		} else comm_to++;

	}

}

// volá se z přerušení
// čtení enkoderů
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


// čtení stavu enkodérů
ISR(TIMER2_COMP_vect) {

	read_enc();

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


// USART - Tx Complete
ISR(USART_UDRE_vect) {

	sendPacket(&UDR,&comm_state);

	if (comm_state.send_state==PS_READY) CLEARBIT(UCSRB,UDRIE);

}

// volá se z main
// vytvoření paketu s info o stavu motorů
void makeMotorInfo() {

	uint8_t data[22]; // 22 bytů


	// ********************************************
	// požadovaná rychlost - stejná pro oba motory
	data[0] = motor1.req_speed;
	data[1] = motor1.req_speed>>8;

	// aktuální rychlost předního motoru
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[2] = motor1.act_speed;
	data[3] = motor1.act_speed>>8; }

	// stav předního motoru
	data[4] = motor1.state;

	// proud předním motorem
	data[5] = motor1.current;

	// teplota předního motoru
	data[6] = motor1.temp;

	// ujetá vzdálenost
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	data[7] = (int32_t)motor1.distance;
	data[8] = (int32_t)motor1.distance>>8;
	data[9] = (int32_t)motor1.distance>>16;
	data[10] = (int32_t)motor1.distance>>24; }

	// výkon předního motoru
	data[11] =  motor1.load;


	// ********************************************
	// aktuální rychlost zadního motoru
	data[12] = motor2.act_speed;
	data[13] = motor2.act_speed>>8;

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

	// výkon zadního motoru
	data[21] = motor2.load;

	makePacket(&comm_state.op,data,22,P_MOTOR_INFO,0);

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

// TODO: otestovat měření proudu
// volá se z main
// měření proudu motory
void motor_current() {


	// spustí převod
	SETBIT(ADCSRA,ADSC);

	// čeká na dokončení převodu
	while (CHECKBIT(ADCSRA,ADSC ));

	motor2.current = (95*motor2.current + 5*((ADCW*10)/64))/100;
	//motor1.current = ADCW/4;

	ADMUX++;

	// spustí převod
	SETBIT(ADCSRA,ADSC);

	// čeká na dokončení převodu
	while (CHECKBIT(ADCSRA,ADSC ));

	motor1.current = (95*motor1.current + 5*((ADCW*10)/64)) / 100;
	//motor2.current = ADCW/4;

	ADMUX--;

}


int main(void) {

	ioinit();


	motor1.state = MOT_RUNNING;
	motor2.state = MOT_RUNNING;

	while(1) {


		// měření proudu motory
		motor_current();

		// přepnutí na příjem
		C_CLEARBIT(RS485_SEND);


		comm_state.receive_state = PR_WAITING;

		// čekání na příjem paketu
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);


		// pokud byl přijat paket -> rozhodnutí podle typu paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

			// indikace příjmu paketu
			C_FLIPBIT(LED);

			// vynulování počítadla pro timeout komunikace -> zastavení motorů
			comm_to = 0;

			switch(comm_state.ip.packet_type) {

			// požadavek na odeslání echo paketu
			case P_ECHO: {

				// vytvoření ECHO paketu
				makePacket(&comm_state.op,comm_state.ip.data,comm_state.ip.len,P_ECHO,0);

				// odeslání paketu
				sendPacketE();

			} break;


			// příjem požadované rychlosti
			case P_MOTOR_COMM: {

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

			// příjem požadované rychlosti
			case P_MOTOR_CLEAR_SUM: {

				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				motor1.sum = 0;
				motor2.sum = 0;
					}


			} break;

			// požadavek na odeslání informací z motorů
			case P_MOTOR_INFO: {

				// vytvoření paketu
				makeMotorInfo();

				// odeslání paketu
				sendPacketE();

			} break;


			// nastavení konstant PID regulátoru
			case P_MOTOR_SETPID: {

				ATOMIC_BLOCK(ATOMIC_FORCEON) {
				motor1.P = comm_state.ip.data[0];
				motor2.P = comm_state.ip.data[0];

				motor1.I = comm_state.ip.data[1];
				motor2.I = comm_state.ip.data[1];

				motor1.D = comm_state.ip.data[2];
				motor2.D = comm_state.ip.data[2];
				}

				// zápis do EEPROM
				eeprom_write_byte(&eP,motor1.P);
				eeprom_write_byte(&eI,motor1.I);
				eeprom_write_byte(&eD,motor1.D);


			} break;

			// odeslání parametrů regulátoru
			case P_MOTOR_GETPID: {

				uint8_t arr[3];

				arr[0] = motor1.P;
				arr[1] = motor1.I;
				arr[2] = motor1.D;

				makePacket(&comm_state.op,arr,3,P_MOTOR_GETPID,0);

				// odeslání paketu
				sendPacketE();


			}

			} // switch


		} // if



		comm_state.receive_state = PR_READY;



	} // while


}
