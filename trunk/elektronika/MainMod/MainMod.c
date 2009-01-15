// MainMod - kód pro øídicí modul
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface


// TODO diagnostické info z modulu
// TODO obsluha komunikace s pc
// TODO joystick - obsluha tlaèítek


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

// povolení vysílání na rs485
#define RS485_SEND PORTD, 4

// pøíjem / vysílání
#define RS485_OUT (C_SETBIT(RS485_SEND))
#define RS485_IN (C_CLEARBIT(RS485_SEND))


// stavové promìnné modulu
volatile tmod_state mod_state;

// stav komunikace - rs485
volatile tcomm_state comm_state;

// stav komunikace - rs232
volatile tcomm_state pccomm_state;


// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

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
	UBRR1L = 3; // 250000
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(1<<TXCIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);


}

inline void set_adc(void) {

	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolení ADC
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

       return acc/MOV;     // nebo acc/N podle potøeby
}

uint16_t moving_average_y(uint16_t value)
{
       static uint16_t buffer[MOV], i=0;
       static uint16_t acc=0;

       acc = acc - buffer[i] + value;
       buffer[i] = value;
       if (++i >= MOV) i = 0;

       return acc/MOV;     // nebo acc/N podle potøeby
}



enum {JOY_Y=6,JOY_X=7};
// spustí AD pøevod pro urèení vychýlení joysticku
uint16_t joystick_xy(uint8_t dir) {



	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitù
	ADMUX |= dir&0x3F; // nastaví kanál

	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	if (dir==JOY_X) return 1023-moving_average_x(ADCW);
	else return moving_average_y(ADCW);
	//return ADCW;

}


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

	for(sent=1; sent<=100;sent++) {

				// vytvoøení paketu
		makePacket(&comm_state.op,data,30,P_ECHO,addr);

		// odeslání paketu
		sendPacketE();

		C_CLEARBIT(RS485_SEND);
		// èekání na odpovìï
		comm_state.receive_state = PR_WAITING;
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_BAD_CRC);

		// crc souhlasí -> úspìšné pøijetí paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED) rec++;

		comm_state.receive_state = PR_READY;

	}

	// 100 pokusù -> rec = úpìšnost v %
	return rec;

}

// TODO: implementovat - pøesunout kód z initModules
void initModule(char *mod_name, uint8_t addr) {




}
// TODO: zobecnit - pro všechny moduly
// inicializace modulù - echo
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

	_delay_ms(2000);
	lcd_clrscr();

	// povolení pøíjmu
	C_CLEARBIT(RS485_SEND);

	sei();

}

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


// vypíše èas na lcd - pravý horní roh
void writeTime() {

	lcd_gotoxy(20-8,0);
	char buff[9];
	uint8_t res;
	res = snprintf(buff,9,"%2u:%2u:%2u",mod_state.hrs,mod_state.min,mod_state.sec);
	lcd_puts(buff);

}

// zobrazí na lcd statistiky komunikace
void commStat(volatile tcomm_state *p) {

	char abuff[10];

	// odeslaných paketù
	lcd_gotoxy(0,1);
	sprintf(abuff,"SE:%7u",(unsigned int)p->packets_sended);
	lcd_puts(abuff);

	// poèet pøijatých paketù
	lcd_gotoxy(0,2);
	sprintf(abuff,"RE:%7u",(unsigned int)p->packets_received);
	lcd_puts(abuff);

	// poèet chyb rámce
	lcd_gotoxy(0,3);
	sprintf(abuff,"FE:%7u",p->frame_error);
	lcd_puts(abuff);

	// poèet paketù s vadným CRC
	lcd_gotoxy(10,1);
	sprintf(abuff,"BR:%7u",p->packets_bad_received);
	lcd_puts(abuff);

	// poèet timeoutù
	lcd_gotoxy(10,2);
	sprintf(abuff,"TO:%7u",p->packets_timeouted);
	lcd_puts(abuff);

}

// zobrazí na LCD stav joysticku
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

		// TODO: tlaèítka
		case M_JOYSTICK: {

			writeTime();
			joy();



		} break;


	} // switch


}


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

	// obnovení lcd
	if (++lcdc==50) {

		lcdc=0;

		// nastavení pøíznakù
		C_SETBIT(MLCD);
		C_SETBIT(MRS232);



	}



}


// odeslání statistiky komunikace s moduly do PC
void sendCommStat() {

	uint8_t data[15]; // 1 byte typ, 14 bytù data
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

	// èekání na odeslání paketu
	while(pccomm_state.send_state != PS_READY);


}


int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	// zjistí dostupnost dalších modulù
	initModules();

	mod_state.menu_state = M_STANDBY;

    while (1) {


    	if (C_CHECKBIT(MLCD)) {

    		C_CLEARBIT(MLCD);

    		// obsluha lcd
    		manageLcd();
    	}

    	// odeslání dat do PC ---------------------------------------------------------------------------
    	if (C_CHECKBIT(MRS232)) {

    	    		C_CLEARBIT(MRS232);

    	    		// odeslání statistiky komunikace s moduly do PC
    	    		sendCommStat(&comm_state,&pccomm_state);

    	    	}

    	// obsluha joysticku ---------------------------------------------------------------------------
    	if (mod_state.menu_state==M_JOYSTICK) {

    		mod_state.joy_x = joystick_xy(JOY_X);
    		mod_state.joy_y = joystick_xy(JOY_Y);

    	}

    	// pøepínání režimu lcd ---------------------------------------------------------------------------
    	if (CHECKBIT(mod_state.buttons,ABUTT1)) {
    		lcd_clrscr();
    		if (++mod_state.menu_state > M_JOYSTICK) mod_state.menu_state = M_STANDBY;
    		CLEARBIT(mod_state.buttons,ABUTT1);

    	}


    	// obsluha komunikace s PC ---------------------------------------------------------------------------
    	pccomm_state.receive_state = PR_WAITING;
    	while(pccomm_state.receive_state != PR_PACKET_RECEIVED && pccomm_state.receive_state!=PR_TIMEOUT && pccomm_state.receive_state!=PR_BAD_CRC);

    	// crc souhlasí -> úspìšné pøijetí paketu
    	if (pccomm_state.receive_state==PR_PACKET_RECEIVED)
    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zpìt stejný paket
    		case P_ECHO: {

    			// TODO: odeslání echa


    		} break;




    		} // switch


    	pccomm_state.receive_state = PR_READY;

    	// obsluha komunikace s moduly ---------------------------------------------------------------------------



    	/*uint8_t data[30], i=0;

    	 for(i=0; i<30;i++) data[i] = i*2;

    	 // vytvoøení paketu
    	    		makePacket(&comm_state.op,data,30,P_ECHO,10);

    	    		// odeslání paketu
    	    		sendPacketE();

    	    		C_CLEARBIT(RS485_SEND);
    	    		// èekání na odpovìï
    	    		comm_state.receive_state = PR_WAITING;
    	    		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT && comm_state.receive_state!=PR_BAD_CRC);

    	    		// crc souhlasí -> úspìšné pøijetí paketu
    	    		//if (comm_state.receive_state==PR_PACKET_RECEIVED) C_FLIPBIT(LCD_BL);

    	    		comm_state.receive_state = PR_READY;


    	    		// vytvoøení paketu - odeslání na neexistující adresu
    	    		  makePacket(&comm_state.op,data,30,P_VALUE,12);

    	    		// odeslání paketu
    	    		  sendPacketE();*/





    }

    return 0;
}
