// MainMod - k�d pro ��dic� modul
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface


// TODO diagnostick� info z modulu
// TODO obsluha komunikace s pc
// TODO joystick - obsluha tla��tek


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


// stavov� prom�nn� modulu
volatile tmod_state mod_state;

// stav komunikace - rs485
volatile tcomm_state comm_state;

// stav komunikace - rs232
volatile tcomm_state pccomm_state;


// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

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
	UBRR1L = 3; // 250000
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(1<<TXCIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);


}

inline void set_adc(void) {

	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolen� ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);


}

#define MOV 4
uint16_t moving_average_x(uint16_t value)
{
       static uint16_t buffer[MOV], i=0;
       static uint16_t acc=0;

       acc = acc - buffer[i] + value;
       buffer[i] = value;
       if (++i >= MOV) i = 0;

       return acc/MOV;     // nebo acc/N podle pot�eby
}

uint16_t moving_average_y(uint16_t value)
{
       static uint16_t buffer[MOV], i=0;
       static uint16_t acc=0;

       acc = acc - buffer[i] + value;
       buffer[i] = value;
       if (++i >= MOV) i = 0;

       return acc/MOV;     // nebo acc/N podle pot�eby
}



enum {JOY_Y=6,JOY_X=7};
// spust� AD p�evod pro ur�en� vych�len� joysticku
uint16_t joystick_xy(uint8_t dir) {



	// nastaven� kan�lu
	ADMUX &= 0xC0; // vynulov�n� 5 spodnich bit�
	ADMUX |= dir&0x3F; // nastav� kan�l

	// spust� p�evod
	SETBIT(ADCSRA,ADSC);

	// �ek� na dokon�en� p�evodu
	while (CHECKBIT(ADCSRA,ADSC ));

	if (dir==JOY_X) return 1023-moving_average_x(ADCW);
	else return moving_average_y(ADCW);
	//return ADCW;

}


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

	for(sent=1; sent<=100;sent++) {

				// vytvo�en� paketu
		makePacket(&comm_state.op,data,30,P_ECHO,addr);

		// odesl�n� paketu
		sendPacketE();

		C_CLEARBIT(RS485_SEND);
		// �ek�n� na odpov��
		comm_state.receive_state = PR_WAITING;
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_BAD_CRC);

		// crc souhlas� -> �sp�n� p�ijet� paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED) rec++;

		comm_state.receive_state = PR_READY;

	}

	// 100 pokus� -> rec = �p�nost v %
	return rec;

}

// TODO: implementovat - p�esunout k�d z initModules
void initModule(char *mod_name, uint8_t addr) {




}
// TODO: zobecnit - pro v�echny moduly
// inicializace modul� - echo
void initModules() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Testing modules...");

	char buff[9];
	uint8_t res,state;

	// MotorControl1
	state = sendEcho(10);

	lcd_gotoxy(0,1);
	if (state>90) lcd_puts_P("MC (10) OK :-)");
	else lcd_puts_P("MC (10) KO :-(");

	lcd_gotoxy(0,2);
	res = snprintf(buff,9,"ECHO:%3u",state);
	lcd_puts(buff);
	lcd_gotoxy(8,2);
	lcd_puts_P("%");

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

	_delay_ms(2000);
	lcd_clrscr();

	// povolen� p��jmu
	C_CLEARBIT(RS485_SEND);

	sei();

}

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


// vyp�e �as na lcd - prav� horn� roh
void writeTime() {

	lcd_gotoxy(20-8,0);
	char buff[9];
	uint8_t res;
	res = snprintf(buff,9,"%2u:%2u:%2u",mod_state.hrs,mod_state.min,mod_state.sec);
	lcd_puts(buff);

}

// zobraz� na lcd statistiky komunikace
void commStat(volatile tcomm_state *p) {

	char abuff[10];

	// odeslan�ch paket�
	lcd_gotoxy(0,1);
	sprintf(abuff,"SE:%7u",(unsigned int)p->packets_sended);
	lcd_puts(abuff);

	// po�et p�ijat�ch paket�
	lcd_gotoxy(0,2);
	sprintf(abuff,"RE:%7u",(unsigned int)p->packets_received);
	lcd_puts(abuff);

	// po�et chyb r�mce
	lcd_gotoxy(0,3);
	sprintf(abuff,"FE:%7u",p->frame_error);
	lcd_puts(abuff);

	// po�et paket� s vadn�m CRC
	lcd_gotoxy(10,1);
	sprintf(abuff,"BR:%7u",p->packets_bad_received);
	lcd_puts(abuff);

	// po�et timeout�
	lcd_gotoxy(10,2);
	sprintf(abuff,"TO:%7u",p->packets_timeouted);
	lcd_puts(abuff);

}

// zobraz� na LCD stav joysticku
void joy() {

	char abuff[10];

	lcd_gotoxy(0,0);
	lcd_puts_P("Joystick");

	// osa x
	lcd_gotoxy(0,1);
	sprintf(abuff,"JX:%7d",mod_state.joy_x);
	lcd_puts(abuff);

	// osa y
	lcd_gotoxy(0,2);
	sprintf(abuff,"JY:%7d",mod_state.joy_y);
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


void manageLcd() {

	switch (mod_state.menu_state) {

		case M_INIT: {


		} break;

		case M_STANDBY: {

			writeTime();
			standBy();

		} break;

		case M_COMMSTAT: {

			writeTime();
			lcd_gotoxy(0,0);
			lcd_puts_P("CommStat");
			commStat(&comm_state);

		} break;

		case M_PCCOMMSTAT: {

			writeTime(&mod_state);
			lcd_gotoxy(0,0);
			lcd_puts_P("PCCommStat");
			commStat(&pccomm_state);

		} break;

		// TODO: tla��tka
		case M_JOYSTICK: {

			writeTime();
			joy();



		} break;


	} // switch


}


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

	// obnoven� lcd
	if (++lcdc==50) {

		lcdc=0;

		// nastaven� p��znak�
		C_SETBIT(MLCD);
		C_SETBIT(MRS232);



	}



}


// odesl�n� statistiky komunikace s moduly do PC
void sendCommStat() {

	uint8_t data[15]; // 1 byte typ, 14 byt� data
	data[0] = COMMSTATE;

	uint8_t index = 1;
	uint8_t i=0;

	for (i=1;i<15;i++)
		data[i]=i*2;

	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_sended>>(i*8));

	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_received>>(i*8));

	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_bad_received>>(i*8));

	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_timeouted>>(i*8));

	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.frame_error>>(i*8));

	makePacket(&pccomm_state.op,data,15,P_VALUE,0);

	sendFirstByte(&UDR0,&pccomm_state);

	// �ek�n� na odesl�n� paketu
	while(pccomm_state.send_state != PS_READY);


}


int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	// zjist� dostupnost dal��ch modul�
	initModules();

	mod_state.menu_state = M_STANDBY;

    while (1) {


    	if (C_CHECKBIT(MLCD)) {

    		C_CLEARBIT(MLCD);

    		// obsluha lcd
    		manageLcd();
    	}

    	if (C_CHECKBIT(MRS232)) {

    		C_CLEARBIT(MRS232);

    		// odesl�n� statistiky komunikace s moduly do PC
    		sendCommStat(&comm_state,&pccomm_state);

    	}

    	uint8_t data[30], i=0;

    		for(i=0; i<30;i++) data[i] = i*2;

    		// vytvo�en� paketu
    		makePacket(&comm_state.op,data,30,P_ECHO,10);

    		// odesl�n� paketu
    		sendPacketE();

    		C_CLEARBIT(RS485_SEND);
    		// �ek�n� na odpov��
    		comm_state.receive_state = PR_WAITING;
    		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_BAD_CRC);

    		// crc souhlas� -> �sp�n� p�ijet� paketu
    		//if (comm_state.receive_state==PR_PACKET_RECEIVED) C_FLIPBIT(LCD_BL);

    		comm_state.receive_state = PR_READY;


    		// vytvo�en� paketu - odesl�n� na neexistuj�c� adresu
    		  makePacket(&comm_state.op,data,30,P_VALUE,12);

    		// odesl�n� paketu
    		  sendPacketE();




    	/*uint8_t data[5]; // 1 byte typ, 14 byt� data
    	data[0] = 0xf1;
    	data[1] = 0xf2;
    	data[2] = 0xf3;
    	data[3] = 0xf4;
    	data[4] = 0xf5;
    	makePacket(&pccomm_state.op,&data,5,P_VAL,0);

    	sendFirstByte(&UDR0,&pccomm_state);

    	// �ek�n� na odesl�n� paketu
    	while(pccomm_state.send_state != PS_READY);*/



    	// obsluha joysticku
    	if (mod_state.menu_state==M_JOYSTICK) {

    		mod_state.joy_x = joystick_xy(JOY_X);
    		mod_state.joy_y = joystick_xy(JOY_Y);

    	}

    	// p�ep�n�n� re�imu lcd
    	if (CHECKBIT(mod_state.buttons,ABUTT1)) {
    		lcd_clrscr();
    		if (++mod_state.menu_state > M_JOYSTICK) mod_state.menu_state = M_STANDBY;
    		CLEARBIT(mod_state.buttons,ABUTT1);

    	}






    }

    return 0;
}
