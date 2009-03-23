// MainMod - k�d pro ��dic� modul
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface


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

// povolen� vys�l�n� na rs485
#define RS485_SEND PORTD, 4

// p��jem / vys�l�n�
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


// *** GLOBALNI PROMENNE **************************************************
// stavov� prom�nn� modulu
volatile tmod_state mod_state;

// stav komunikace - rs485
volatile tcomm_state comm_state;

// stav komunikace - rs232
volatile tcomm_state pccomm_state;

// prom�nn� pro motory
// lf = left front, lr = left rear
// rf = right front, rr = right rear
volatile tmotor m_lf,m_lr,m_rf,m_rr;


// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

// stav senzor�
volatile tsens sens;


#define MLCD flags, 0
#define MRS232 flags, 1


// nastaven� obou UART�
inline void set_uarts() {

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

	// povolen� ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


}



// vol� se z main
// odesl�n� paketu - extra funkce pro ka�d� modul
void sendPacketE() {

	// zak�z�n� p��jmu
	CLEARBIT(UCSR1B,RXEN1);

	// p�epnut� na vys�l�n�
	C_SETBIT(RS485_SEND);

	// nastaven� 9. bitu
	SETBIT(UCSR1B,TXB81);

	// posl�n� prvn�ho bytu - ostatn� se vys�laj� automaticky
	sendFirstByte(&UDR1,&comm_state);

	// �ek�n� na odesl�n� paketu
	while(comm_state.send_state != PS_READY);

	// p�epnut� na p��jem
	C_CLEARBIT(RS485_SEND);

	// povolen� p��jmu
	SETBIT(UCSR1B,RXEN1);

}

// prov��� komunikaci s modulem zadan� adresy
// vrac� �sp�nost v %
uint8_t sendEcho(uint8_t addr) {

	uint8_t data[30], i=0,sent=0,rec=0;

	for(i=0; i<30;i++) data[i] = i*2;

	for(sent=1; sent<=10;sent++) {

		// vytvo�en� paketu
		makePacket(&comm_state.op,data,30,P_ECHO,addr);

		// odesl�n� paketu
		sendPacketE();

		C_CLEARBIT(RS485_SEND);
		// �ek�n� na odpov��
		comm_state.receive_state = PR_WAITING;
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT);

		// crc souhlas� -> �sp�n� p�ijet� paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) rec++;

		comm_state.receive_state = PR_READY;

	}

	// 10 pokus� -> rec*10 = �p�nost v %
	return rec*10;

}


// inicializace modul� - echo
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

	// PORTD5-7 tla��tka, aktivovan� pullupy, PD3 = TXD1, PD4 = RE/DE
	DDRD = (0<<DDD5)|(0<<DDD6)|(0<<DDD7)|(1<<DDD3)|(1<<DDD4);
	PORTD = (1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7)|(1<<PORTD3)|(1<<PORTD4);

	C_CLEARBIT(LCD_BL);

	lcd_init(LCD_DISP_ON);
	lcd_home();

	lcd_puts_P("Starting-up...");


	// CT0 - asynch. -> lcd menu, tlacitka, hodiny, regulace podsvetleni lcd
		// 10 Hz - N=32, OCR=50
		// 100 Hz - N=1, OCR=163

		// asynchronn� taktov�n�
		ASSR = (1<<AS0);

		// 100 Hz
		OCR0 = 163;
		TCCR0 = (0<<CS02)|(0<<CS01)|(1<<CS00);

		// OCIE0: Timer/Counter0 Output Compare Match Interrupt Enable
		TIMSK = (1<<OCIE0);

	// nastaven� obou UART�
	set_uarts();

	// nastaven� ADC
	set_adc();

	// inicializace struktury comm_state
	comm_state_init(&comm_state);
	comm_state_init(&pccomm_state);

	// inic. struktur motor�
	motor_init(&m_lf);
	motor_init(&m_lr);
	motor_init(&m_rf);
	motor_init(&m_rr);

	_delay_ms(2100);
	lcd_clrscr();

	// povolen� p��jmu
	C_CLEARBIT(RS485_SEND);

	sei();

}

// vol� se z p�eru�en�
// �ten� a zpracov�n� stavu tla��tek
void checkButtons () {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocn� prom�nn� pro o�et�en� z�kmit� tla��tek bez �ek�n�
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

// vol� se z p�eru�en�
// po��tadlo pro �as
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

// vol� se z main
// vyp�e �as na lcd - prav� horn� roh
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

// zobraz� na lcd statistiky komunikace
void commStat(volatile tcomm_state *p) {

	char abuff[11];

	// odeslan�ch paket�
	lcd_gotoxy(0,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SE:%7u"),p->packets_sended);}
	lcd_puts(abuff);

	// po�et p�ijat�ch paket�
	lcd_gotoxy(0,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("RE:%7u"),p->packets_received);}
	lcd_puts(abuff);

	// po�et chyb r�mce
	lcd_gotoxy(0,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("FE:%7u"),p->frame_error);}
	lcd_puts(abuff);

	// po�et paket� s vadn�m CRC
	lcd_gotoxy(10,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("BR:%7u"),p->packets_bad_received);}
	lcd_puts(abuff);

	// po�et timeout�
	lcd_gotoxy(10,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("TO:%7u"),p->packets_timeouted);}
	lcd_puts(abuff);

	// chyba synchronizace
	lcd_gotoxy(10,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SY:%7u"),p->sync_error);}
	lcd_puts(abuff);

}

// vol� se z main
// zobraz� na LCD stav joysticku
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

// stand. re�im displeje
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

	// po odvys�l�n� adresy zru�it nastaven� 9. bit
	if (comm_state.send_state>=PS_ADDR) CLEARBIT(UCSR1B,TXB81);
	else SETBIT(UCSR1B,TXB81);

	sendPacket(&UDR1,&comm_state);



}

// vol� se z main
// na LCD zobraz� info z motoru
void motStat(volatile tmotor *m) {

	char abuff[11];

	// po�. rychlost
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

	// ujet� vzd�lenost
	lcd_gotoxy(10,3);
	sprintf_P(abuff,PSTR("DI:%7d"),m->distance);
	lcd_puts(abuff);


}

// p�eru�en� - 50 Hz
ISR(TIMER0_COMP_vect) {


	// po��tadlo pro obnoven� lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	// aktualizace vnitrniho casu
	updateTime();

	// kontrola tla��tek
	checkButtons();

	// po��tadlo timeoutu pro p��jem po RS485
	receiveTimeout(&comm_state);

	// po��tadlo timeoutu pro p��jem po RS232
	receiveTimeout(&pccomm_state);

	// 1 Hz
	if (++lcdc==50) {

		lcdc=0;

		// zv��en� po��tadla pro timeout komunikace s PC
		//if (mod_state.pc_comm_to<PcCommTo) mod_state.pc_comm_to++;

		// nastaven� p��znak�
		C_SETBIT(MLCD);
		C_SETBIT(MRS232);



	}



}

// vol� se z main
// odesl�n� statistiky komunikace s moduly do PC
void sendCommStat() {

	uint8_t data[15]; // 1 byte typ, 14 byt� data
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

	// �ek�n� na odesl�n� paketu
	while(pccomm_state.send_state != PS_READY);


}

// vol� se z main
// dek�duje p�ijat� info z modulu MotorControl
void decodeMotorInfo(volatile tmotor *mf, volatile tmotor *mr) {

		// �daje pro p�edn� motor ----------------------------------

		// po�adovan� rychlost
		mf->req_speed = comm_state.ip.data[0];
		mf->req_speed |= ((int16_t)comm_state.ip.data[1])<<8;

		// aktu�ln� rychlost
		mf->act_speed = comm_state.ip.data[2];
		mf->act_speed |= (int16_t)comm_state.ip.data[3]<<8;

		// stav motoru
		mf->state = comm_state.ip.data[4];

		// proud motoru
		mf->current = comm_state.ip.data[5];

		// teplota motoru
		mf->temp = comm_state.ip.data[6];

		// ujet� vzd�lenost
		mf->distance = comm_state.ip.data[7];
		mf->distance  |= (int32_t)comm_state.ip.data[8]<<8;
		mf->distance  |= (int32_t)comm_state.ip.data[9]<<16;
		mf->distance  |= (int32_t)comm_state.ip.data[10]<<24;

		// v�kon motoru
		mf->load = comm_state.ip.data[11];


		// �daje pro zadn� motor -------------------------------------------------

		// po�adovan� rychlost
		mr->req_speed = comm_state.ip.data[0];
		mr->req_speed |= (int16_t)comm_state.ip.data[1]<<8;

		// aktu�ln� rychlost
		mr->act_speed = comm_state.ip.data[12];
		mr->act_speed |= (int16_t)comm_state.ip.data[13]<<8;

		// stav motoru
		mr->state = comm_state.ip.data[14];

		// proud motoru
		mr->current = comm_state.ip.data[15];

		// teplota motoru
		mr->temp = comm_state.ip.data[16];

		// ujet� vzd�lenost
		mr->distance = comm_state.ip.data[17];
		mr->distance  |= (int32_t)comm_state.ip.data[18]<<8;
		mr->distance  |= (int32_t)comm_state.ip.data[19]<<16;
		mr->distance  |= (int32_t)comm_state.ip.data[20]<<24;

		// v�kon motoru
		mr->load = comm_state.ip.data[21];

}

// spust� AD p�evod pro ur�en� vych�len� joysticku
void update_joystick(volatile tmod_state *m) {

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

// zobraz� informace ze senzor� na LCD
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

// zobraz� informace ze senzor� na LCD
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

// zobrazen� PID konstant na lcd
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

	   // z�kladn� obrazovka
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

	    // lev� p�edn� motor
	    case M_MLF: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftFront");
	    	motStat(&m_lf);

	    } break;

	    // lev� zadn� motor
	    case M_MLR: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftRear");
	    	motStat(&m_lr);


	    } break;


	    // prav� p�edn� motor
	    case M_MRF: {

	    	writeTime(&mod_state);
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightFront");
	    	motStat(&m_rf);

	    } break;

	    // prav� zadn� motor
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

	    // zobrazen� stavu joysticku
	    case M_JOYSTICK: {

	    	writeTime();
	    	joy();

	    } break;


	    		} // switch



}

// na�te z modulu info o motorech a vypln� ho do struktur tmotor
void getMotorInfo(uint8_t addr, volatile tmotor *front, volatile tmotor *rear) {

	// �ten� stavu lev�ch motor�
	makePacket(&comm_state.op,NULL,0,P_MOTOR_INFO,addr);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// �ek�n� na odpov��
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlas� -> �sp�n� p�ijet� paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

	   // dek�dov�n� p�ijat�ch dat
	   decodeMotorInfo(front,rear);

	   };

	 comm_state.receive_state = PR_READY;


}


// na�te z modulu SensMod
void getFastSensorState() {

	// �ten� stavu lev�ch motor�
	makePacket(&comm_state.op,NULL,0,P_SENS_FAST,21);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// �ek�n� na odpov��
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlas� -> �sp�n� p�ijet� paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

	   // data ze sonaru
	   sens.us_fast = comm_state.ip.data[0];
	   sens.us_fast |= comm_state.ip.data[1]<<8;

	   // lev� p�edn� sharp
	   sens.sharp[0] = comm_state.ip.data[2];
	   sens.sharp[0] |= comm_state.ip.data[3]<<8;

	   // prav� p�edn� sharp
	   sens.sharp[1] = comm_state.ip.data[4];
	   sens.sharp[1] |= comm_state.ip.data[5]<<8;

	   // lev� zadn� sharp
	   sens.sharp[2] = comm_state.ip.data[6];
	   sens.sharp[2] |= comm_state.ip.data[7]<<8;

	   // prav� zadn� sharp
	   sens.sharp[3] = comm_state.ip.data[8];
	   sens.sharp[3] |= comm_state.ip.data[9]<<8;

	   // taktiln� senzory
	   sens.tact = comm_state.ip.data[10];


	   };

	 comm_state.receive_state = PR_READY;


}

// provede pln� skenov�n� a na�te data ze SensMod
void getFullSensorState() {

	// �ten� stavu lev�ch motor�
	makePacket(&comm_state.op,NULL,0,P_SENS_FULL,21);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// TODO: vyzkou�et, jestli je 1500ms dost
	// �ek�n� na dokon�en� m��en�
	//_delay_ms(1500);

	// �ek�n� na odpov��
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_READY);

	// crc souhlas� -> �sp�n� p�ijet� paketu
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

	   // lev� p�edn� sharp
	   sens.sharp[0] = comm_state.ip.data[10];
	   sens.sharp[0] |= comm_state.ip.data[11]<<8;

	   // prav� p�edn� sharp
	   sens.sharp[1] = comm_state.ip.data[12];
	   sens.sharp[1] |= comm_state.ip.data[13]<<8;

	   // lev� zadn� sharp
	   sens.sharp[2] = comm_state.ip.data[14];
	   sens.sharp[2] |= comm_state.ip.data[15]<<8;

	   // prav� zadn� sharp
	   sens.sharp[3] = comm_state.ip.data[16];
	   sens.sharp[3] |= comm_state.ip.data[17]<<8;

	   // taktiln� senzory
	   sens.tact = comm_state.ip.data[18];


	   };

	 comm_state.receive_state = PR_READY;


}

enum {O_FRONT, O_REAR};
// zji��uje, jestli nen� n�jak� p�ek�ka p��li� bl�zko
// v p��pad�, �e ano, vrac� 1
uint8_t checkNearObstacle(uint8_t where) {

	// definov�n� bezpe�n� vzd�lenosti (Sharpy nezm��� m��, �e 50)
	#define DIS 100

	switch (where) {

	case O_FRONT: {

		// test p�edn�ch Sharp� (0 indikuje p�ek�ku mimo rozsah). ultrazvuku
		if ((sens.sharp[0]<=DIS && sens.sharp[0]!=0) || (sens.sharp[1]<=DIS && sens.sharp[1]!=0) || (sens.us_fast<=DIS && sens.us_fast!=0)) return 1;
		else return 0;


	} break;

	case O_REAR: {

		// kontrola zadn�ch Sharp�
		if ((sens.sharp[2]<=DIS && sens.sharp[2]!=0) || (sens.sharp[3]<=DIS && sens.sharp[3]!=0)) return 1;
		else return 0;

	} break;



	}

	return 1;

}

// nastaven� po�adovan� rychlosti motor�
void setMotorSpeed(uint8_t addr, int16_t speed) {

	uint8_t sarr[2];

	sarr[0] = speed;
	sarr[1] = speed>>8;

	makePacket(&comm_state.op,sarr,2,P_MOTOR_COMM,addr);

	sendPacketE();


}

// zji�t�n� PID konstant
void getMotorPID(uint8_t addr) {


	// �ten� stavu lev�ch motor�
	makePacket(&comm_state.op,NULL,0,P_MOTOR_GETPID,addr);

	sendPacketE();

	C_CLEARBIT(RS485_SEND);

	// �ek�n� na odpov��
	comm_state.receive_state = PR_WAITING;
	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_READY);

	// crc souhlas� -> �sp�n� p�ijet� paketu
	if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) {

		mP = comm_state.ip.data[0];
		mI = comm_state.ip.data[1];
		mD = comm_state.ip.data[2];

	}

	comm_state.receive_state = PR_READY;


}

// nastaven� PID parametr�
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

	// zjist� dostupnost dal��ch modul�
	initModules();

	mod_state.menu_state = M_STANDBY;

	// nastaven� zdroje ��zen� na PC
	mod_state.control = C_PC;

	// zji�t�n� PID konstant nastaven�ch v EEPROM
	getMotorPID(10);

	// nekone�n� smy�ka
    while (1) {

    	// p�ep�n�n� re�imu lcd ---------------------------------------------------------------------------
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

    	// TODO: nastaven� zdroje ��zen� C_AUTO, C_JOY, C_PC

    	if (CHECKBIT(mod_state.buttons,ABUTT3)) {

    		if (++mod_state.control>C_PC) mod_state.control = C_AUTO;

    	    CLEARBIT(mod_state.buttons,ABUTT3);

    	}



    	// obsluha LCD
    	if (C_CHECKBIT(MLCD)) {

    		C_CLEARBIT(MLCD);
    		manageLcd();


    	}


    	// obsluha periferi� a pod��zen�ch modul� ---------------------------------------------------------------------------

    	update_joystick(&mod_state);

		// �ten� stavu lev�ch motor�
		getMotorInfo(10,&m_lf,&m_lr);

    	// �ten� stavu prav�ch motor�
		getMotorInfo(11,&m_rf,&m_rr);

		// na�ten� dat ze senzor�
		getFastSensorState();


		switch(mod_state.control) {

		// zdroj ��zen� nastaven na PC
		case C_PC: {

			// zastaven� p�i timeoutu komunikace s PC - 5s
			/*if (mod_state.pc_comm_to >= PcCommTo) {

				// po�. rychlost nen� nula - zastavit
				if (m_lf.req_speed!=0 || m_rf.req_speed!=0) {

					setMotorSpeed(10,0);
					setMotorSpeed(11,0);

				}

			}*/


		} break;

		// zdroj ��zen� nastaven na joystick
		case C_JOY: {

			// TODO: dod�lat zat��en�

			int16_t sp = 0, ot = 0;

			sp = (int16_t)(mod_state.joy_y-511)/2;

			ot = (int16_t)((1022-mod_state.joy_x)-511)/2;

			// pr�h - p��li� n�zk� hodnoty se neuva�uj�
			if (sp > -5 && sp < 5) sp = 0;
			if (ot > -5 && ot < 5) ot = 0;

			// jedeme rovn�
			if ((ot>-200) && (ot<200)) {

			// kontrola p�ek�ek vp�edu
			if (sp>0 && checkNearObstacle(O_FRONT)) sp = 0;

			// kontrola p�ek�ek vzadu
			if (sp<0 && checkNearObstacle(O_REAR)) sp = 0;

			setMotorSpeed(10,sp);
			setMotorSpeed(11,sp);

			// zat��en� doleva
			} else if (ot<=-200) {

				setMotorSpeed(10,-sp);
				setMotorSpeed(11,sp);

			// zat��en� doprava
			} else if (ot>=200) {

				setMotorSpeed(10,sp);
				setMotorSpeed(11,-sp);

			}


		} break;

		// autonomn� operace
		case C_AUTO: {

			// TODO: n�hodn� proj��ka

		} break;




		} //switch



    	// obsluha komunikace s PC ---------------------------------------------------------------------------

    	// TODO: timeout pro komunikaci s PC -> zastaven�, kdy� dlouho nic nep�ijde

		pccomm_state.receive_state = PR_WAITING;

		while (pccomm_state.receive_state!=PR_PACKET_RECEIVED && pccomm_state.receive_state!=PR_TIMEOUT && pccomm_state.receive_state!=PR_READY);

    	// crc souhlas� -> �sp�n� p�ijet� paketu
    	if ((pccomm_state.receive_state==PR_PACKET_RECEIVED) && checkPacket(&pccomm_state)) {

    		// vynulov�n� po��tadla timeoutu kom. s PC
    		mod_state.pc_comm_to = 0;

    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zp�t stejn� paket
    		case P_ECHO: {

    			// �ek�n� na p��padn� dokon�en� odesl�n� p�edchoz�ho paketu
    			while(pccomm_state.send_state != PS_READY);

    			// vytvo�en� ECHO paketu
    			makePacket(&pccomm_state.op,pccomm_state.ip.data,pccomm_state.ip.len,P_ECHO,0);

    			// zah�jen� p�enosu
    			sendFirstByte(&UDR0,&pccomm_state);


    		} break;

    		// voln� j�zda - ovl�d�n� joystickem
    		case PC_FREE_RIDE: {

    			uint16_t spl, spr;

    			if (mod_state.control == C_PC ) {

					// rychlost pro lev� motory
					spl = pccomm_state.ip.data[0];
					spl |= pccomm_state.ip.data[1]<<8;

					// rychlost pro prav� motory
					spr = pccomm_state.ip.data[2];
					spr |= pccomm_state.ip.data[3]<<8;

					// kontrola p�ek�ek vp�edu
					if (spl>0 && spr>0 && checkNearObstacle(O_FRONT)) {

						spl = 0;
						spr = 0;
					}

					// kontrola p�ek�ek vzadu
					if (spl<0 && spr<0 && checkNearObstacle(O_REAR)) {

						spl = 0;
						spr = 0;

					}

					setMotorSpeed(10,spl);
					setMotorSpeed(11,spr);

    			}

    		} break;

    		// po�adavek na nastaven� PID konstant
    		case P_MOTOR_SETPID: {

    			mP = pccomm_state.ip.data[0];
    			mI = pccomm_state.ip.data[1];
    			mD = pccomm_state.ip.data[2];

    			setMotorPID(10);
    			setMotorPID(11);


    		} break;

    		// po�adavek na p�e�ten� PID konstant
    		case P_MOTOR_GETPID: {

    			uint8_t arr[3];

    			arr[0] = mP;
    		    arr[1] = mI;
    		    arr[2] = mD;

    		    makePacket(&pccomm_state.op,arr,3,P_MOTOR_GETPID,0);

    		    // zah�jen� p�enosu
    		   sendFirstByte(&UDR0,&pccomm_state);

    		   // �ek�n� na p��padn� dokon�en� odesl�n� p�edchoz�ho paketu
    		   while(pccomm_state.send_state != PS_READY);


    		} break;




    		// j�zda rovn�
    		case PC_MOVE_STRAIGHT: {

    			// TODO: naprogramovat
    			if (mod_state.control == C_PC ) {





    			}


    		} break;

    		// oto�en�
    		case PC_MOVE_ROUND: {

    		    // TODO: naprogramovat
    			if (mod_state.control == C_PC ) {



    			}


    		} break;

    		// informace o stavu pohon�
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

				// zah�jen� p�enosu
				sendFirstByte(&UDR0,&pccomm_state);

				// �ek�n� na p��padn� dokon�en� odesl�n� p�edchoz�ho paketu
				while(pccomm_state.send_state != PS_READY);


    		} break;

    		case PC_MOVE_AINFO: {

    			// TODO: dod�lat

    		} break;

    		// data ze senzor� - m��eno za pohybu
    		case P_SENS_FAST: {

    			uint8_t arr[11];

    			// ultrazvuk - rychl� m��en�
    			arr[0] = sens.us_fast;
    			arr[1] = sens.us_fast>>8;

    			// sharp 1 (lev� p�edn�)
    			arr[2] = sens.sharp[0];
    			arr[3] = sens.sharp[0]>>8;

    			// sharp 2 (prav� p�edn�)
    			arr[4] = sens.sharp[1];
    			arr[5] = sens.sharp[1]>>8;

    			// sharp 3 (lev� zadn�)
    			arr[6] = sens.sharp[2];
    			arr[7] = sens.sharp[2]>>8;

    			// sharp 4 (prav� zadn�)
    			arr[8] = sens.sharp[3];
    			arr[9] = sens.sharp[3]>>8;

    			// taktiln� senzory
    			arr[10] = sens.tact;


    			// vytvo�en� paketu
    			makePacket(&pccomm_state.op,arr,11,P_SENS_FAST,0);

    			// zah�jen� p�enosu
    			sendFirstByte(&UDR0,&pccomm_state);

    			// �ek�n� na p��padn� dokon�en� odesl�n� p�edchozho paketu
    			while(pccomm_state.send_state != PS_READY);

    		} break;

    		// pln� m��en� - pouze kdy� se stoj�
    		case P_SENS_FULL: {

    			// provede se pouze kdy� robot stoj� a zdroj ��zen� je nastaven� na PC
    			if ((m_lf.act_speed == 0) && (m_rf.act_speed == 0) && (mod_state.control == C_PC)) {

    				// prov�st pln� skenov�n�
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


					// sharp 1 (lev� p�edn�)
    				arr[10] = sens.sharp[0];
    				arr[11] = sens.sharp[0]>>8;

    				// sharp 2 (prav� p�edn�)
    				arr[12] = sens.sharp[1];
    				arr[13] = sens.sharp[1]>>8;

    				// sharp 3 (lev� zadn�)
    				arr[14] = sens.sharp[2];
    				arr[15] = sens.sharp[2]>>8;

    				// sharp 4 (prav� zadn�)
    				arr[16] = sens.sharp[3];
    				arr[17] = sens.sharp[3]>>8;

    				// taktiln� senzory
    				arr[18] = sens.tact;

    				// �ek�n� na p��padn� dokon�en� odesl�n� p�edchozho paketu
    				while(pccomm_state.send_state != PS_READY);

    				// vytvo�en� paketu
    				makePacket(&pccomm_state.op,arr,11,P_SENS_FAST,0);

    				// zah�jen� p�enosu
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
