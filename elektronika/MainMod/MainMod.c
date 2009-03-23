// MainMod - kód pro øídicí modul
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface


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

#include "lib/lcd.h"
#include "MainMod.h"
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

#define LCD_BL  PORTG, 1
#define BUTT4 PIND, 5
#define BUTT3 PIND, 6
#define BUTT2 PIND, 7
#define BUTT1 PING, 0

// povolení vysílání na rs485
#define RS485_SEND PORTD, 4

// pøíjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


// *** GLOBALNI PROMENNE **************************************************
// stavové promìnné modulu
volatile tmod_state mod_state;

// stav komunikace - rs485
volatile tcomm_state comm_state;

// stav komunikace - rs232
volatile tcomm_state pccomm_state;

// promìnné pro motory
// lf = left front, lr = left rear
// rf = right front, rr = right rear
volatile tmotor m_lf,m_lr,m_rf,m_rr;


// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

// stav senzorù
volatile tsens sens;


#define MLCD flags, 0
#define MRS232 flags, 1


// nastavení obou UARTù
inline void set_uarts() {

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
	UCSR0B = (0<<UCSZ02)|(1<<TXCIE0)|(1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (0<<USBS0)|(0<<UMSEL0)|(0<<UPM01)|(0<<UPM00)|(1<<UCSZ01)|(1<<UCSZ00);

	// RS485, 9n2
	// 9600bd - UBRR1L = 103;
	//UBRR1L = 68; // 14400
	//UBRR1L = 25; // 38400
	//UBRR1L = 12; // 76800
	UBRR1L = 12;
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(1<<TXCIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);


}

// inicializace struktur
void motor_init(volatile tmotor *m) {

	m->act_speed = 0;
	m->current = 0;
	m->distance = 0;
	m->req_speed = 0;
	m->state = 0;
	m->temp = 0;
	m->load = 0;

}

inline void set_adc(void) {

	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolení ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


}



// volá se z main
// odeslání paketu - extra funkce pro každý modul
void sendPacketE() {

	// zakázání pøíjmu
	CLEARBIT(UCSR1B,RXEN1);

	// pøepnutí na vysílání
	C_SETBIT(RS485_SEND);

	// nastavení 9. bitu
	SETBIT(UCSR1B,TXB81);

	// poslání prvního bytu - ostatní se vysílají automaticky
	sendFirstByte(&UDR1,&comm_state);

	// èekání na odeslání paketu
	while(comm_state.send_state != PS_READY);

	// pøepnutí na pøíjem
	C_CLEARBIT(RS485_SEND);

	// povolení pøíjmu
	SETBIT(UCSR1B,RXEN1);

}

// provìøí komunikaci s modulem zadané adresy
// vrací úspìšnost v %
uint8_t sendEcho(uint8_t addr) {

	uint8_t data[30], i=0,sent=0,rec=0;

	for(i=0; i<30;i++) data[i] = i*2;

	for(sent=1; sent<=10;sent++) {

		// vytvoøení paketu
		makePacket(&comm_state.op,data,30,P_ECHO,addr);

		// odeslání paketu
		sendPacketE();

		C_CLEARBIT(RS485_SEND);
		// èekání na odpovìï
		comm_state.receive_state = PR_WAITING;
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT);

		// crc souhlasí -> úspìšné pøijetí paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) rec++;

		comm_state.receive_state = PR_READY;

	}

	// 10 pokusù -> rec*10 = úpìšnost v %
	return rec*10;

}


// inicializace modulù - echo
void initModules() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Testing modules...");

	char buff[15];
	uint8_t state;

	// MotorControl - left
	state = sendEcho(10);

	lcd_gotoxy(0,1);
	snprintf_P(buff,15,PSTR("MCL (10) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// MotorControl - right
	state = sendEcho(11);

	lcd_gotoxy(0,2);
	snprintf_P(buff,15,PSTR("MCR (11) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// SensMod
	state = sendEcho(21);

	lcd_gotoxy(0,3);
	snprintf_P(buff,15,PSTR("SEM (21) %3u\%"),state);
	lcd_puts(buff);


	_delay_ms(2000);
	lcd_clrscr();



}

inline void ioinit(void) {

	DDRA = (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3)|(1<<DDA4)|(1<<DDA5)|(1<<DDA6);
	PORTA = (1<<PORTA0)|(1<<PORTA1)|(1<<PORTA2)|(1<<PORTA3)|(1<<PORTA4)|(1<<PORTA5)|(1<<PORTA6);

	DDRG = (0<<DDG0)|(1<<DDG1);
	PORTG = (1<<PORTG0)|(1<<PORTG1);

	// PE1 = TXD0
	DDRE = (1<<DDE1);
	PORTE = (1<<PORTE1);

	// PORTD5-7 tlaèítka, aktivované pullupy, PD3 = TXD1, PD4 = RE/DE
	DDRD = (0<<DDD5)|(0<<DDD6)|(0<<DDD7)|(1<<DDD3)|(1<<DDD4);
	PORTD = (1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7)|(1<<PORTD3)|(1<<PORTD4);

	C_CLEARBIT(LCD_BL);

	lcd_init(LCD_DISP_ON);
	lcd_home();

	lcd_puts_P("Starting-up...");


	// CT0 - asynch. -> lcd menu, tlacitka, hodiny, regulace podsvetleni lcd
		// 10 Hz - N=32, OCR=50
		// 100 Hz - N=1, OCR=163

		// asynchronní taktování
		ASSR = (1<<AS0);

		// 100 Hz
		OCR0 = 163;
		TCCR0 = (0<<CS02)|(0<<CS01)|(1<<CS00);

		// OCIE0: Timer/Counter0 Output Compare Match Interrupt Enable
		TIMSK = (1<<OCIE0);

	// nastavení obou UARTù
	set_uarts();

	// nastavení ADC
	set_adc();

	// inicializace struktury comm_state
	comm_state_init(&comm_state);
	comm_state_init(&pccomm_state);

	// inic. struktur motorù
	motor_init(&m_lf);
	motor_init(&m_lr);
	motor_init(&m_rf);
	motor_init(&m_rr);

	_delay_ms(2100);
	lcd_clrscr();

	// povolení pøíjmu
	C_CLEARBIT(RS485_SEND);

	sei();

}

// volá se z pøerušení
// ètení a zpracování stavu tlaèítek
void checkButtons () {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocné promìnné pro ošetøení zákmitù tlaèítek bez èekání
	static uint8_t pbutt1=0, pbutt2=0, pbutt3=0, pbutt4=0;

	if (!C_CHECKBIT(BUTT1) && ++pbutt1==BUTT_DEBOUNCE) {
		//pbutt1 = 0;
		SETBIT(mod_state.buttons,ABUTT1);
	}
	if (C_CHECKBIT(BUTT1)) pbutt1=0;


	if (!C_CHECKBIT(BUTT2) && ++pbutt2==BUTT_DEBOUNCE) {
		//pbutt2 = 0;
		SETBIT(mod_state.buttons,ABUTT2);
	}
	if (C_CHECKBIT(BUTT2)) pbutt2=0;

	if (!C_CHECKBIT(BUTT3) && ++pbutt3==BUTT_DEBOUNCE) {
		//pbutt3 = 0;
		SETBIT(mod_state.buttons,ABUTT3);
	}
	if (C_CHECKBIT(BUTT3)) pbutt3=0;

	if (!C_CHECKBIT(BUTT4) && ++pbutt4==BUTT_DEBOUNCE) {
		//pbutt4 = 0;
		SETBIT(mod_state.buttons,ABUTT4);
	}
	if (C_CHECKBIT(BUTT4)) pbutt4=0;

}

// volá se z pøerušení
// poèítadlo pro èas
void updateTime()
{


	if (mod_state.msec+=10, mod_state.msec==1000) {
		mod_state.msec = 0;
		if (++mod_state.sec==60) {
			mod_state.sec=0;
			if (++mod_state.min==60) {
				mod_state.min = 0;
				if (++mod_state.hrs==24) mod_state.hrs=0;
			}
		}
	}


}

// volá se z main
// vypíše èas na lcd - pravý horní roh
void writeTime() {

	char ctrl=' ';

	switch(mod_state.control) {

	case C_AUTO: ctrl = 'A'; break;
	case C_JOY: ctrl = 'J'; break;
	case C_PC: ctrl = 'P'; break;


	}

	lcd_gotoxy(20-9,0);
	char buff[10];
	snprintf(buff,10,"%c%2u:%2u:%2u",ctrl,mod_state.hrs,mod_state.min,mod_state.sec);
	lcd_puts(buff);

}

// zobrazí na lcd statistiky komunikace
void commStat(volatile tcomm_state *p) {

	char abuff[11];

	// odeslaných paketù
	lcd_gotoxy(0,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SE:%7u"),p->packets_sended);}
	lcd_puts(abuff);

	// poèet pøijatých paketù
	lcd_gotoxy(0,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("RE:%7u"),p->packets_received);}
	lcd_puts(abuff);

	// poèet chyb rámce
	lcd_gotoxy(0,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("FE:%7u"),p->frame_error);}
	lcd_puts(abuff);

	// poèet paketù s vadným CRC
	lcd_gotoxy(10,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("BR:%7u"),p->packets_bad_received);}
	lcd_puts(abuff);

	// poèet timeoutù
	lcd_gotoxy(10,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("TO:%7u"),p->packets_timeouted);}
	lcd_puts(abuff);

	// chyba synchronizace
	lcd_gotoxy(10,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SY:%7u"),p->sync_error);}
	lcd_puts(abuff);

}

// volá se z main
// zobrazí na LCD stav joysticku
void joy() {

	char abuff[11];

	lcd_gotoxy(0,0);
	lcd_puts_P("Joystick");

	// osa x
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("JX:%7d"),mod_state.joy_x);
	lcd_puts(abuff);

	// osa y
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("JY:%7d"),mod_state.joy_y);
	lcd_puts(abuff);


}

// stand. režim displeje
void standBy() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Standby");


}



// USART0 - komunikace s PC
ISR(USART0_RX_vect) {

	receivePacket(UDR0,&pccomm_state);

	if (CHECKBIT(UCSR0A,FE0)) pccomm_state.frame_error++;

}

// USART0 - Tx Complete
ISR(USART0_TX_vect) {

	sendPacket(&UDR0,&pccomm_state);

}

// USART1 - Rx Complete
ISR(USART1_RX_vect) {

	receivePacket(UDR1,&comm_state);
	if (CHECKBIT(UCSR1A,FE1)) comm_state.frame_error++;



}


// USART1 - Tx Complete
ISR(USART1_TX_vect) {

	// po odvysílání adresy zrušit nastavený 9. bit
	if (comm_state.send_state>=PS_ADDR) CLEARBIT(UCSR1B,TXB81);
	else SETBIT(UCSR1B,TXB81);

	sendPacket(&UDR1,&comm_state);



}

// volá se z main
// na LCD zobrazí info z motoru
void motStat(volatile tmotor *m) {

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

// pøerušení - 50 Hz
ISR(TIMER0_COMP_vect) {


	// poèítadlo pro obnovení lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	// aktualizace vnitrniho casu
	updateTime();

	// kontrola tlaèítek
	checkButtons();

	// poèítadlo timeoutu pro pøíjem po RS485
	receiveTimeout(&comm_state);

	// poèítadlo timeoutu pro pøíjem po RS232
	receiveTimeout(&pccomm_state);

	// 1 Hz
	if (++lcdc==50) {

		lcdc=0;

		// zvýšení poèítadla pro timeout komunikace s PC
		//if (mod_state.pc_comm_to<PcCommTo) mod_state.pc_comm_to++;

		// nastavení pøíznakù
		C_SETBIT(MLCD);
		C_SETBIT(MRS232);



	}



}

// volá se z main
// odeslání statistiky komunikace s moduly do PC
void sendCommStat() {

	uint8_t data[15]; // 1 byte typ, 14 bytù data
	data[0] = COMMSTATE;

	uint8_t index = 1;
	uint8_t i=0;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_sended>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_bad_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_timeouted>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.frame_error>>(i*8)); }

	makePacket(&pccomm_state.op,data,15,P_COMM_INFO,0);

	sendFirstByte(&UDR0,&pccomm_state);

	// èekání na odeslání paketu
	while(pccomm_state.send_state != PS_READY);


}

// volá se z main
// dekóduje pøijaté info z modulu MotorControl
void decodeMotorInfo(volatile tmotor *mf, volatile tmotor *mr) {

		// údaje pro pøední motor ----------------------------------

		// požadovaná rychlost
		mf->req_speed = comm_state.ip.data[0];
		mf->req_speed |= ((int16_t)comm_state.ip.data[1])<<8;

		// aktuální rychlost
		mf->act_speed = comm_state.ip.data[2];
		mf->act_speed |= (int16_t)comm_state.ip.data[3]<<8;

		// stav motoru
		mf->state = comm_state.ip.data[4];

		// proud motoru
		mf->current = comm_state.ip.data[5];

		// teplota motoru
		mf->temp = comm_state.ip.data[6];

		// ujetá vzdálenost
		mf->distance = comm_state.ip.data[7];
		mf->distance  |= (int32_t)comm_state.ip.data[8]<<8;
		mf->distance  |= (int32_t)comm_state.ip.data[9]<<16;
		mf->distance  |= (int32_t)comm_state.ip.data[10]<<24;

		// výkon motoru
		mf->load = comm_state.ip.data[11];


		// údaje pro zadní motor -------------------------------------------------

		// požadovaná rychlost
		mr->req_speed = comm_state.ip.data[0];
		mr->req_speed |= (int16_t)comm_state.ip.data[1]<<8;

		// aktuální rychlost
		mr->act_speed = comm_state.ip.data[12];
		mr->act_speed |= (int16_t)comm_state.ip.data[13]<<8;

		// stav motoru
		mr->state = comm_state.ip.data[14];

		// proud motoru
		mr->current = comm_state.ip.data[15];

		// teplota motoru
		mr->temp = comm_state.ip.data[16];

		// ujetá vzdálenost
		mr->distance = comm_state.ip.data[17];
		mr->distance  |= (int32_t)comm_state.ip.data[18]<<8;
		mr->distance  |= (int32_t)comm_state.ip.data[19]<<16;
		mr->distance  |= (int32_t)comm_state.ip.data[20]<<24;

		// výkon motoru
		mr->load = comm_state.ip.data[21];

}

// spustí AD pøevod pro urèení vychýlení joysticku
void update_joystick(volatile tmod_state *m) {

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

// zobrazí informace ze senzorù na LCD
void sensInfo() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),sens.us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("UF:%7u"),sens.us_fast);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("S1:%7u"),sens.sharp[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("S2:%7u"),sens.sharp[1]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("S3:%7u"),sens.sharp[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("S4:%7u"),sens.sharp[3]);
	lcd_puts(abuff);



}

// zobrazí informace ze senzorù na LCD
void sensFullInfo() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("U1:%7u"),sens.us_full[0]);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("U2:%7u"),sens.us_full[1]);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("U3:%7u"),sens.us_full[2]);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("U4:%7u"),sens.us_full[3]);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("U5:%7u"),sens.us_full[4]);
	lcd_puts(abuff);


}

// zobrazení PID konstant na lcd
void motPID() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("P:%7u"),mP);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("I:%7u"),mI);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("D:%7u"),mD);
	lcd_puts(abuff);

}

// obsluha LCD
void manageLcd() {

	// obsluha lcd
	switch (mod_state.menu_state) {

	   case M_INIT: {


	   } break;

	   // základní obrazovka
	   case M_STANDBY: {

		   writeTime();
	    	standBy();

	    } break;

	    // statistiky komunikace s moduly
	    case M_COMMSTAT: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("CommStat");
	    	commStat(&comm_state);

	    } break;

	    // stat. komunikace s PC
	    case M_PCCOMMSTAT: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("PCCommStat");
	    	commStat(&pccomm_state);

	    } break;

	    // levý pøední motor
	    case M_MLF: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftFront");
	    	motStat(&m_lf);

	    } break;

	    // levý zadní motor
	    case M_MLR: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftRear");
	    	motStat(&m_lr);


	    } break;


	    // pravý pøední motor
	    case M_MRF: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightFront");
	    	motStat(&m_rf);

	    } break;

	    // pravý zadní motor
	    case M_MRR: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightRear");
	    	motStat(&m_rr);


	    } break;

	    case M_PID: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("PIDconst");
	    	motPID();

	    }

	    case M_SENS: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("Sensors");
	    	sensInfo();

	    } break;

	    case M_SENS_FULL: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("FSensors");
	    	sensFullInfo();

	    	    } break;

	    // zobrazení stavu joysticku
	    case M_JOYSTICK: {

	    	writeTime();
	    	joy();

	    } break;


	    		} // switch



}

// naète z modulu info o motorech a vyplní ho do struktur tmotor
void getMotorInfo(uint8_t addr, volatile tmotor *front, volatile tmotor *rear) {

	// ètení stavu levých motorù
	makePacket(&comm_state.op,NULL,0,P_MOTOR_INFO,addr);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

	   // dekódování pøijatých dat
	   decodeMotorInfo(front,rear);

	   };

	 comm_state.receive_state = PR_READY;


}


// naète z modulu SensMod
void getFastSensorState() {

	// ètení stavu levých motorù
	makePacket(&comm_state.op,NULL,0,P_SENS_FAST,21);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

	   // data ze sonaru
	   sens.us_fast = comm_state.ip.data[0];
	   sens.us_fast |= comm_state.ip.data[1]<<8;

	   // levý pøední sharp
	   sens.sharp[0] = comm_state.ip.data[2];
	   sens.sharp[0] |= comm_state.ip.data[3]<<8;

	   // pravý pøední sharp
	   sens.sharp[1] = comm_state.ip.data[4];
	   sens.sharp[1] |= comm_state.ip.data[5]<<8;

	   // levý zadní sharp
	   sens.sharp[2] = comm_state.ip.data[6];
	   sens.sharp[2] |= comm_state.ip.data[7]<<8;

	   // pravý zadní sharp
	   sens.sharp[3] = comm_state.ip.data[8];
	   sens.sharp[3] |= comm_state.ip.data[9]<<8;

	   // taktilní senzory
	   sens.tact = comm_state.ip.data[10];


	   };

	 comm_state.receive_state = PR_READY;


}

// provede plné skenování a naète data ze SensMod
void getFullSensorState() {

	// ètení stavu levých motorù
	makePacket(&comm_state.op,NULL,0,P_SENS_FULL,21);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// TODO: vyzkoušet, jestli je 1500ms dost
	// èekání na dokonèení mìøení
	//_delay_ms(1500);

	// èekání na odpovìï
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

	   // data ze sonaru
	   // 0st
	   sens.us_full[0] = comm_state.ip.data[0];
	   sens.us_full[0] |= comm_state.ip.data[1]<<8;

	   // 45st
	   sens.us_full[1] = comm_state.ip.data[2];
	   sens.us_full[1] |= comm_state.ip.data[3]<<8;

	   // 90st
	   sens.us_full[2] = comm_state.ip.data[4];
	   sens.us_full[2] |= comm_state.ip.data[5]<<8;

	   // 135st
	   sens.us_full[3] = comm_state.ip.data[6];
	   sens.us_full[3] |= comm_state.ip.data[7]<<8;

	   // 180st
	   sens.us_full[4] = comm_state.ip.data[8];
	   sens.us_full[4] |= comm_state.ip.data[9]<<8;

	   // levý pøední sharp
	   sens.sharp[0] = comm_state.ip.data[10];
	   sens.sharp[0] |= comm_state.ip.data[11]<<8;

	   // pravý pøední sharp
	   sens.sharp[1] = comm_state.ip.data[12];
	   sens.sharp[1] |= comm_state.ip.data[13]<<8;

	   // levý zadní sharp
	   sens.sharp[2] = comm_state.ip.data[14];
	   sens.sharp[2] |= comm_state.ip.data[15]<<8;

	   // pravý zadní sharp
	   sens.sharp[3] = comm_state.ip.data[16];
	   sens.sharp[3] |= comm_state.ip.data[17]<<8;

	   // taktilní senzory
	   sens.tact = comm_state.ip.data[18];


	   };

	 comm_state.receive_state = PR_READY;


}

enum {O_FRONT, O_REAR};
// zjišuje, jestli není nìjaká pøekážka pøíliš blízko
// v pøípadì, že ano, vrací 1
uint8_t checkNearObstacle(uint8_t where) {

	// definování bezpeèné vzdálenosti (Sharpy nezmìøí míò, že 50)
	#define DIS 100

	switch (where) {

	case O_FRONT: {

		// test pøedních Sharpù (0 indikuje pøekážku mimo rozsah). ultrazvuku
		if ((sens.sharp[0]<=DIS && sens.sharp[0]!=0) || (sens.sharp[1]<=DIS && sens.sharp[1]!=0) || (sens.us_fast<=DIS && sens.us_fast!=0)) return 1;
		else return 0;


	} break;

	case O_REAR: {

		// kontrola zadních Sharpù
		if ((sens.sharp[2]<=DIS && sens.sharp[2]!=0) || (sens.sharp[3]<=DIS && sens.sharp[3]!=0)) return 1;
		else return 0;

	} break;



	}

	return 1;

}

// nastavení požadované rychlosti motorù
void setMotorSpeed(uint8_t addr, int16_t speed) {

	uint8_t sarr[2];

	sarr[0] = speed;
	sarr[1] = speed>>8;

	makePacket(&comm_state.op,sarr,2,P_MOTOR_COMM,addr);

	sendPacketE();


}

// zjištìní PID konstant
void getMotorPID(uint8_t addr) {


	// ètení stavu levých motorù
	makePacket(&comm_state.op,NULL,0,P_MOTOR_GETPID,addr);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// èekání na odpovìï
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlasí -> úspìšné pøijetí paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

		mP = comm_state.ip.data[0];
		mI = comm_state.ip.data[1];
		mD = comm_state.ip.data[2];

	}

	comm_state.receive_state = PR_READY;


}

// nastavení PID parametrù
void setMotorPID(uint8_t addr) {

	uint8_t sarr[3];

	sarr[0] = mP;
	sarr[1] = mI;
	sarr[2] = mD;

	makePacket(&comm_state.op,sarr,3,P_MOTOR_SETPID,addr);

	sendPacketE();


}


int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	// zjistí dostupnost dalších modulù
	initModules();

	mod_state.menu_state = M_STANDBY;

	// nastavení zdroje øízení na PC
	mod_state.control = C_PC;

	// zjištìní PID konstant nastavených v EEPROM
	getMotorPID(10);

	// nekoneèná smyèka
    while (1) {

    	// pøepínání režimu lcd ---------------------------------------------------------------------------
    	if (CHECKBIT(mod_state.buttons,ABUTT1)) {
    	    lcd_clrscr();

    	    ATOMIC_BLOCK(ATOMIC_FORCEON) {
    	    if (++mod_state.menu_state > M_JOYSTICK) mod_state.menu_state = M_STANDBY;
    	    CLEARBIT(mod_state.buttons,ABUTT1);
    	    }

    	    }

    	if (CHECKBIT(mod_state.buttons,ABUTT2)) {
    	    lcd_clrscr();

    	    ATOMIC_BLOCK(ATOMIC_FORCEON) {
    	    if (mod_state.menu_state > M_STANDBY) mod_state.menu_state--;
    	    else mod_state.menu_state = M_JOYSTICK;

    	    CLEARBIT(mod_state.buttons,ABUTT2);
    	    }

    	}

    	// TODO: nastavení zdroje øízení C_AUTO, C_JOY, C_PC

    	if (CHECKBIT(mod_state.buttons,ABUTT3)) {

    		if (++mod_state.control>C_PC) mod_state.control = C_AUTO;

    	    CLEARBIT(mod_state.buttons,ABUTT3);

    	}



    	// obsluha LCD
    	if (C_CHECKBIT(MLCD)) {

    		C_CLEARBIT(MLCD);
    		manageLcd();


    	}


    	// obsluha periferií a podøízených modulù ---------------------------------------------------------------------------

    	update_joystick(&mod_state);

		// ètení stavu levých motorù
		getMotorInfo(10,&m_lf,&m_lr);

    	// ètení stavu pravých motorù
		getMotorInfo(11,&m_rf,&m_rr);

		// naètení dat ze senzorù
		getFastSensorState();


		switch(mod_state.control) {

		// zdroj øízení nastaven na PC
		case C_PC: {

			// zastavení pøi timeoutu komunikace s PC - 5s
			/*if (mod_state.pc_comm_to >= PcCommTo) {

				// pož. rychlost není nula - zastavit
				if (m_lf.req_speed!=0 || m_rf.req_speed!=0) {

					setMotorSpeed(10,0);
					setMotorSpeed(11,0);

				}

			}*/


		} break;

		// zdroj øízení nastaven na joystick
		case C_JOY: {

			// TODO: dodìlat zatáèení

			int16_t sp = 0, ot = 0;

			sp = (int16_t)(mod_state.joy_y-511)/2;

			ot = (int16_t)((1022-mod_state.joy_x)-511)/2;

			// práh - pøíliš nízké hodnoty se neuvažují
			if (sp > -5 && sp < 5) sp = 0;
			if (ot > -5 && ot < 5) ot = 0;

			// jedeme rovnì
			if ((ot>-200) && (ot<200)) {

			// kontrola pøekážek vpøedu
			if (sp>0 && checkNearObstacle(O_FRONT)) sp = 0;

			// kontrola pøekážek vzadu
			if (sp<0 && checkNearObstacle(O_REAR)) sp = 0;

			setMotorSpeed(10,sp);
			setMotorSpeed(11,sp);

			// zatáèení doleva
			} else if (ot<=-200) {

				setMotorSpeed(10,-sp);
				setMotorSpeed(11,sp);

			// zatáèení doprava
			} else if (ot>=200) {

				setMotorSpeed(10,sp);
				setMotorSpeed(11,-sp);

			}


		} break;

		// autonomní operace
		case C_AUTO: {

			// TODO: náhodná projížïka

		} break;




		} //switch



    	// obsluha komunikace s PC ---------------------------------------------------------------------------

    	// TODO: timeout pro komunikaci s PC -> zastavení, když dlouho nic nepøijde

		pccomm_state.receive_state = PR_WAITING;

		while (pccomm_state.receive_state!=PR_PACKET_RECEIVED && pccomm_state.receive_state!=PR_TIMEOUT && pccomm_state.receive_state!=PR_READY);

    	// crc souhlasí -> úspìšné pøijetí paketu
    	if ((pccomm_state.receive_state==PR_PACKET_RECEIVED) && checkPacket(&pccomm_state)) {

    		// vynulování poèítadla timeoutu kom. s PC
    		mod_state.pc_comm_to = 0;

    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zpìt stejný paket
    		case P_ECHO: {

    			// èekání na pøípadné dokonèení odeslání pøedchozího paketu
    			while(pccomm_state.send_state != PS_READY);

    			// vytvoøení ECHO paketu
    			makePacket(&pccomm_state.op,pccomm_state.ip.data,pccomm_state.ip.len,P_ECHO,0);

    			// zahájení pøenosu
    			sendFirstByte(&UDR0,&pccomm_state);


    		} break;

    		// volná jízda - ovládání joystickem
    		case PC_FREE_RIDE: {

    			uint16_t spl, spr;

    			if (mod_state.control == C_PC ) {

					// rychlost pro levé motory
					spl = pccomm_state.ip.data[0];
					spl |= pccomm_state.ip.data[1]<<8;

					// rychlost pro pravé motory
					spr = pccomm_state.ip.data[2];
					spr |= pccomm_state.ip.data[3]<<8;

					// kontrola pøekážek vpøedu
					if (spl>0 && spr>0 && checkNearObstacle(O_FRONT)) {

						spl = 0;
						spr = 0;
					}

					// kontrola pøekážek vzadu
					if (spl<0 && spr<0 && checkNearObstacle(O_REAR)) {

						spl = 0;
						spr = 0;

					}

					setMotorSpeed(10,spl);
					setMotorSpeed(11,spr);

    			}

    		} break;

    		// požadavek na nastavení PID konstant
    		case P_MOTOR_SETPID: {

    			mP = pccomm_state.ip.data[0];
    			mI = pccomm_state.ip.data[1];
    			mD = pccomm_state.ip.data[2];

    			setMotorPID(10);
    			setMotorPID(11);


    		} break;

    		// požadavek na pøeètení PID konstant
    		case P_MOTOR_GETPID: {

    			uint8_t arr[3];

    			arr[0] = mP;
    		    arr[1] = mI;
    		    arr[2] = mD;

    		    makePacket(&pccomm_state.op,arr,3,P_MOTOR_GETPID,0);

    		    // zahájení pøenosu
    		   sendFirstByte(&UDR0,&pccomm_state);

    		   // èekání na pøípadné dokonèení odeslání pøedchozího paketu
    		   while(pccomm_state.send_state != PS_READY);


    		} break;




    		// jízda rovnì
    		case PC_MOVE_STRAIGHT: {

    			// TODO: naprogramovat
    			if (mod_state.control == C_PC ) {





    			}


    		} break;

    		// otoèení
    		case PC_MOVE_ROUND: {

    		    // TODO: naprogramovat
    			if (mod_state.control == C_PC ) {



    			}


    		} break;

    		// informace o stavu pohonù
    		case PC_MOVE_INFO: {

				int16_t aspeedl,aspeedr, rspeedl, rspeedr;
				int32_t distl,distr;

				aspeedl = (m_lf.act_speed + m_lr.act_speed)/2;
				rspeedl = (m_lf.req_speed + m_lr.req_speed)/2;
				distl = (m_lf.distance + m_lr.distance)/2;

				aspeedr = (m_rf.act_speed + m_rr.act_speed)/2;
				rspeedr = (m_rf.req_speed + m_rr.req_speed)/2;
				distr = (m_rf.distance + m_rr.distance)/2;

				uint8_t arr[16];

				arr[0] = aspeedl;
				arr[1] = aspeedl>>8;

				arr[2] = rspeedl;
				arr[3] = rspeedl>>8;

				arr[4] = distl;
				arr[5] = distl>>8;
				arr[6] = distl>>16;
				arr[7] = distl>>24;

				arr[8] = aspeedr;
				arr[9] = aspeedr>>8;

				arr[10] = rspeedr;
				arr[11] = rspeedr>>8;

				arr[12] = distr;
				arr[13] = distr>>8;
				arr[14] = distr>>16;
				arr[15] = distr>>24;


				makePacket(&pccomm_state.op,arr,16,P_MOTOR_INFO,0);

				// zahájení pøenosu
				sendFirstByte(&UDR0,&pccomm_state);

				// èekání na pøípadné dokonèení odeslání pøedchozího paketu
				while(pccomm_state.send_state != PS_READY);


    		} break;

    		case PC_MOVE_AINFO: {

    			// TODO: dodìlat

    		} break;

    		// data ze senzorù - mìøeno za pohybu
    		case P_SENS_FAST: {

    			uint8_t arr[11];

    			// ultrazvuk - rychlé mìøení
    			arr[0] = sens.us_fast;
    			arr[1] = sens.us_fast>>8;

    			// sharp 1 (levý pøední)
    			arr[2] = sens.sharp[0];
    			arr[3] = sens.sharp[0]>>8;

    			// sharp 2 (pravý pøední)
    			arr[4] = sens.sharp[1];
    			arr[5] = sens.sharp[1]>>8;

    			// sharp 3 (levý zadní)
    			arr[6] = sens.sharp[2];
    			arr[7] = sens.sharp[2]>>8;

    			// sharp 4 (pravý zadní)
    			arr[8] = sens.sharp[3];
    			arr[9] = sens.sharp[3]>>8;

    			// taktilní senzory
    			arr[10] = sens.tact;


    			// vytvoøení paketu
    			makePacket(&pccomm_state.op,arr,11,P_SENS_FAST,0);

    			// zahájení pøenosu
    			sendFirstByte(&UDR0,&pccomm_state);

    			// èekání na pøípadné dokonèení odeslání pøedchozho paketu
    			while(pccomm_state.send_state != PS_READY);

    		} break;

    		// plné mìøení - pouze když se stojí
    		case P_SENS_FULL: {

    			// provede se pouze když robot stojí a zdroj øízení je nastavený na PC
    			if ((m_lf.act_speed == 0) && (m_rf.act_speed == 0) && (mod_state.control == C_PC)) {

    				// provést plné skenování
    				getFullSensorState();

    				uint8_t arr[19];

    				// us - 0st
    				arr[0] = sens.us_full[0];
    				arr[1] = sens.us_full[0]>>8;

    				// us - 45st
    				arr[2] = sens.us_full[1];
    				arr[3] = sens.us_full[1]>>8;

    				// us - 90st
    				arr[4] = sens.us_full[2];
    				arr[5] = sens.us_full[2]>>8;

    				// us - 135st
    				arr[6] = sens.us_full[3];
    				arr[7] = sens.us_full[3]>>8;

    				// us - 180st
    				arr[8] = sens.us_full[4];
    				arr[9] = sens.us_full[4]>>8;


					// sharp 1 (levý pøední)
    				arr[10] = sens.sharp[0];
    				arr[11] = sens.sharp[0]>>8;

    				// sharp 2 (pravý pøední)
    				arr[12] = sens.sharp[1];
    				arr[13] = sens.sharp[1]>>8;

    				// sharp 3 (levý zadní)
    				arr[14] = sens.sharp[2];
    				arr[15] = sens.sharp[2]>>8;

    				// sharp 4 (pravý zadní)
    				arr[16] = sens.sharp[3];
    				arr[17] = sens.sharp[3]>>8;

    				// taktilní senzory
    				arr[18] = sens.tact;

    				// èekání na pøípadné dokonèení odeslání pøedchozho paketu
    				while(pccomm_state.send_state != PS_READY);

    				// vytvoøení paketu
    				makePacket(&pccomm_state.op,arr,11,P_SENS_FAST,0);

    				// zahájení pøenosu
    				sendFirstByte(&UDR0,&pccomm_state);

    			}



    		} break;


    		} // switch

    		pccomm_state.receive_state = PR_WAITING;

    	} // if


    	if ((pccomm_state.receive_state == PR_TIMEOUT) || (pccomm_state.receive_state == PR_READY)) pccomm_state.receive_state = PR_WAITING;



    	//_delay_ms(100);

    } // while

    return 0;
}
