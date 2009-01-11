// MainMod - kód pro øídicí modul
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface


// TODO diagnostické info z modulu
// TODO obsluha komunikace s pc
// TODO joystick
// TODO menu

// jde o øídicí modul
#define MAINMOD

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


// stav modulu - menu, tlaèítka, èas atd.
typedef struct {

	// stavy menu
	enum {M_STANDBY,M_COMMSTAT,M_JOYSTICK};

	// aktuální stav menu
	volatile uint8_t menu_state;

	// poèítadlo pro krátké zapnutí podsvìtlení po stisku tlaèítka
	volatile uint16_t backlight;

	// realny cas (nebo uptime?)
	volatile uint8_t hrs, min, sec;
	volatile uint16_t msec;

	// stav joysticku
	volatile uint16_t joy_x, joy_y;

	// stav tlaèítek - po pøeètení vynulovat
	uint8_t buttons;
	#define ABUTT1 buttons, 0
	#define ABUTT2 buttons, 1
	#define ABUTT3 buttons, 2
	#define ABUTT4 buttons, 3




} tmod_state;

// globální promìnné
// stavové promìnné modulu
volatile static tmod_state mod_state;

// stav komunikace
volatile static tcomm_state comm_state;

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

	// RS232, 38,4k, 8n1
	UBRR0L = 25;
	UBRR0H = 0;

	UCSR0A = (0<<U2X0)|(0<<MPCM0);
	UCSR0B = (0<<UCSZ02)|(1<<TXCIE0)|(1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (0<<USBS0)|(0<<UMSEL0)|(0<<UPM01)|(0<<UPM00)|(1<<UCSZ01)|(1<<UCSZ00);

	// RS485
	// 9600bd - UBRR1L = 103;
	//UBRR1L = 68; // 14400
	//UBRR1L = 25; // 38400
	//UBRR1L = 12; // 76800
	UBRR1L = 3; // 250000
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (0<<UCSZ12)|(1<<TXCIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
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



enum {JOY_X=7,JOY_Y=6};
// spustí AD pøevod pro urèení vychýlení joysticku
uint16_t joystick_xy(uint8_t dir) {



	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitù
	ADMUX |= dir&0x3F; // nastaví kanál AD6

	// spustí pøevod
	SETBIT(ADCSRA,ADSC);

	// èeká na dokonèení pøevodu
	while (CHECKBIT(ADCSRA,ADSC ));

	if (dir==JOY_X) return moving_average_x(ADCW);
	else return moving_average_y(ADCW);
	//return ADCW;

}

inline void ioinit(void) {

	DDRA = (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3)|(1<<DDA4)|(1<<DDA5)|(1<<DDA6);
	PORTA = (1<<PORTA0)|(1<<PORTA1)|(1<<PORTA2)|(1<<PORTA3)|(1<<PORTA4)|(1<<PORTA5)|(1<<PORTA6);

	DDRG = (0<<DDG0)|(1<<DDG1);
	PORTG = (1<<PORTG0)|(1<<PORTG1);

	// PORTD5-7 tlaèítka, aktivované pullupy, PD3 = TXD1, PD4 = RE/DE
	DDRD = (0<<DDD5)|(0<<DDD6)|(0<<DDD7)|(1<<DDD3)|(1<<DDD4);
	PORTD = (1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7)|(1<<PORTD3)|(1<<PORTD4);

	C_CLEARBIT(LCD_BL);

	lcd_init(LCD_DISP_ON);
	lcd_home();
	lcd_puts_P("RHI - inicializace");
	//lcd_gotoxy (uint8_t x, uint8_t y)

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

		// stav modulu
		tmod_state mod_state;

		// povolení vysílání
		C_SETBIT(RS485_SEND);

	_delay_ms(1000);
	lcd_clrscr();


	sei();

}

// USART0 - komunikace s PC
ISR(USART0_RX_vect) {

	C_FLIPBIT(LCD_BL);

}

// USART0 - Tx Complete
ISR(USART0_TX_vect) {



}

// USART1 - Rx Complete
ISR(USART1_RX_vect) {

	uint8_t byte = UDR1;
	//if (!CHECKBIT(UCSR1A,FE1)) receivePacket(byte,&comm_state);
	//else comm_state.frame_error++;

	// TODO: zrušení pøíjmu pøi frame error
	receivePacket(byte,&comm_state);
	if (CHECKBIT(UCSR1A,FE1)) comm_state.frame_error++;

	// bliknout pøi synchronizaèních bytech
	/*static uint8_t last;
	if (last==SYNC1 && byte == SYNC2) C_FLIPBIT(LCD_BL);
	last = byte;*/

}


// USART1 - Tx Complete
ISR(USART1_TX_vect) {

	sendPacket(&UDR1,&comm_state);

}

// TODO: dodìlat checkButtons (nastavovani bitù, noautorepeat)
void checkButtons () {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocné promìnné pro ošetøení zákmitù tlaèítek bez èekání
	static uint8_t pbutt1=0, pbutt2=0, pbutt3=0, pbutt4=0;

	if (!C_CHECKBIT(BUTT1) && ++pbutt1==BUTT_DEBOUNCE) {
		pbutt1 = 0;
		lcd_puts_P("tlacitko 1");
	}
	if (C_CHECKBIT(BUTT1)) pbutt1=0;


	if (!C_CHECKBIT(BUTT2) && ++pbutt2==BUTT_DEBOUNCE) {
		pbutt2 = 0;
		lcd_puts_P("tlacitko 2");
	}
	if (C_CHECKBIT(BUTT2)) pbutt2=0;

	if (!C_CHECKBIT(BUTT3) && ++pbutt3==BUTT_DEBOUNCE) {
		pbutt3 = 0;
		lcd_puts_P("tlacitko 3");
	}
	if (C_CHECKBIT(BUTT3)) pbutt3=0;

	if (!C_CHECKBIT(BUTT4) && ++pbutt4==BUTT_DEBOUNCE) {
		pbutt4 = 0;
		lcd_puts_P("tlacitko 4");
	}
	if (C_CHECKBIT(BUTT4)) pbutt4=0;

}

void updateTime(tmod_state *p) {

	if (p!=NULL) {
		if (p->msec+=10, p->msec==1000) {
			p->msec = 0;
			if (++p->sec==60) {
				p->sec=0;
				if (++p->min==60) {
					p->min = 0;
					if (++p->hrs==24) p->hrs=0;
				}
			}
		}
	}

}

char *getTimeString(tmod_state *p) {

	char *buff;
	buff = NULL;
	uint8_t res;

	buff = malloc(9);
	if (buff!=NULL && p!=NULL) {

		res = snprintf(buff,9,"%2i:%2i:%2i",p->hrs,p->min,p->sec);
		return buff;

	} else return NULL;



}

// vypíše èas na lcd - pravý horní roh
void writeTime(void) {

	lcd_gotoxy(20-8,0);
	char *buff;
	buff = getTimeString(&mod_state);
	lcd_puts(buff);
	free(buff);

}

// zobrazí na lcd statistiky komunikace
void commStat(void) {

	char abuff[10];

	lcd_gotoxy(0,0);
	lcd_puts_P("CommStat");

	// odeslaných paketù
	lcd_gotoxy(0,1);
	sprintf(abuff,"SE:%7d",comm_state.packets_sended);
	lcd_puts(abuff);

	// poèet pøijatých paketù
	lcd_gotoxy(0,2);
	sprintf(abuff,"RE:%7d",comm_state.packets_received);
	lcd_puts(abuff);

	// poèet chyb rámce
	lcd_gotoxy(0,3);
	sprintf(abuff,"FE:%7d",comm_state.frame_error);
	lcd_puts(abuff);

	// poèet paketù s vadným CRC
	lcd_gotoxy(10,1);
	sprintf(abuff,"BR:%7d",comm_state.packets_bad_received);
	lcd_puts(abuff);

	// poèet timeoutù
	lcd_gotoxy(10,2);
	sprintf(abuff,"TO:%7d",comm_state.packets_timeouted);
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

ISR(TIMER0_COMP_vect) {


	// poèítadlo pro obnovení lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	// aktualizace vnitrniho casu
	updateTime(&mod_state);

	// poèítadlo timeoutu pro pøíjem po RS485
	receiveTimeout(&comm_state);

	// obnovení lcd
	if (++lcdc==50) {

		lcdc=0;


		switch (mod_state.menu_state) {

		case M_STANDBY: {

			writeTime();

		} break;

		case M_COMMSTAT: {

			writeTime();
			commStat();

		} break;

		case M_JOYSTICK: {

			writeTime();
			joy();



		} break;


		}



	}



}




int main(void)
{

	ioinit();

	C_CLEARBIT(LCD_BL);

	C_CLEARBIT(RS485_SEND);

	mod_state.menu_state = M_JOYSTICK;

    while (1) {

    	mod_state.joy_x = joystick_xy(JOY_X);
    	mod_state.joy_y = joystick_xy(JOY_Y);


    	uint8_t data[30], i;

    	for(i=0; i<30;i++) data[i] = i*2;

    	// vytvoøení paketu
    	makePacket(&comm_state.op,&data,30,P_ECHO,10);


    	// zakázání pøíjmu
    	CLEARBIT(UCSR1B,RXEN1);

    	// pøepnutí na vysílání
    	C_SETBIT(RS485_SEND);

    	// poslání prvního bytu - ostatní se vysílají automaticky
    	sendFirstByte(&UDR1,&comm_state);

    	// èekání na odeslání paketu
    	while(comm_state.send_state != PS_READY);

    	// pøepnutí na pøíjem
    	C_CLEARBIT(RS485_SEND);

    	// povolení pøíjmu
    	 SETBIT(UCSR1B,RXEN1);

    	comm_state.send_state=PS_READY;

    	// èekání na odpovìï
    	while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT);
    	comm_state.receive_state = PR_READY;


    }

    return 0;
}
