#define F_CPU 16000000UL  // 16 MHz

#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>

#include "uart.h"


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

uint8_t ena_st=0;

#define PEN1A ena_st, 0
#define PEN1B ena_st, 1

#define PEN2A ena_st, 2
#define PEN2B ena_st, 3


volatile int32_t t_en1=0, pt_en1 = 0, t_en2 = 0, pt_en2 = 0;
volatile int16_t diff1 = 0, diff2 = 0;

//volatile int16_t e;
volatile static int32_t sum;
volatile int16_t act;
volatile int16_t speed;

volatile int16_t e[5];


#define UART_BAUD_RATE  115200





static inline void
ioinit (void) {

	DDRD = (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6);
	PORTD = (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6);
	
	DDRB = (1<<DDB1)|(1<<DDB2);
	PORTB = (1<<PORTB1)|(1<<DDB2);
	
	DDRC = (0<<DDC2)|(0<<DDC3)|(0<<DDC4)|(0<<DDC5);
	PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5);
	
	
	uart_init( UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU) );
	
	// ct0, 40kHz, 8x presc, normal mode
	TCCR0 = (0<<CS02)|(1<<CS01)|(0<<CS00);
	TCNT0 = 50;
	
	// 100 Hz
	//ICR1 = 1250; //  PWM period - TOP
	ICR1 = 1000;
	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); // phase correct, 10-bit
	TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10) | (0<<WGM13) | (0<<WGM12); // 64x presc.
	
	OCR1A = 0;
	
	OCR1B = 0;
	
	/*C_SETBIT(INPUT4);
	C_CLEARBIT(INPUT3);*/
	
	
	// 100 Hz -> OCR2=78, N=1024, CTC mode
	TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(1<<CS22)|(1<<CS21)|(0<<CS20);
	OCR2 = 78;
	ASSR = (0<<AS2);
	
	
	TIMSK = (1<<TOIE0)|(1<<TOIE1);
	
	sei();

}

/* regulator */
ISR(TIMER1_OVF_vect) {

	static uint8_t pocit = 0;

	
	
	// 10 Hz
	if (++pocit==10) {


		//C_FLIPBIT(LED);
		
		diff1 = t_en1 - pt_en1;
		diff2 = t_en2 - pt_en2;
		
		pt_en1 = t_en1;
		pt_en2 = t_en2;
		
		pocit = 0;
		
		
		//odchylka = reference - rychlost;
		//akce = P*odchylka + I*suma;
		//suma = suma + odchylka;
		
		
		
		
		
		
	
	}
	
		
		// klouzavy prumer odchylky
		e[0] = e[1];
		e[1] = e[2];
		e[2] = e[3];
		e[4] = (speed-diff2);
		
		int16_t pe = (e[0] + e[1] + e[2] + e[3] + e[4])/5;
		
		act = ((int16_t)(4*pe+0.9*sum)); 
		
		if (act>0) {
		
			C_SETBIT(INPUT4);
			C_CLEARBIT(INPUT3);
		
		} else {
		
			C_SETBIT(INPUT3);
			C_CLEARBIT(INPUT4);
		
		}

		if (abs(act)<=ICR1) OCR1B = abs(act);
			else OCR1B = ICR1;
		
		sum += pe;
		if (sum > 3000) sum = 3000;
		
	

}


/* Vzorkovani stavu enkoderu */
ISR(TIMER0_OVF_vect) {

	

	// vzestupna hrana - EN2A
		if ((C_CHECKBIT(EN2A)) && (!C_CHECKBIT(PEN2A))) {
			if (C_CHECKBIT(EN2B)) t_en2++;
				else t_en2--;
		} else
		// sestupna hrana - EN2A
		if ((!C_CHECKBIT(EN2A)) && (C_CHECKBIT(PEN2A))) {
			if (!C_CHECKBIT(EN2B)) t_en2++;
				else t_en2--;
		}
		
		
		
		
		// vzestupna hrana - EN2B
		if ((C_CHECKBIT(EN2B)) && (!C_CHECKBIT(PEN2B))) {
			if (!C_CHECKBIT(EN2A)) t_en2++;
				else t_en2--;
		} else
		// sestupna hrana - EN2B
		if ((!C_CHECKBIT(EN2B)) && (C_CHECKBIT(PEN2B))) {
			if (C_CHECKBIT(EN2A)) t_en2++;
				else t_en2--;
		}
		
		
		// ulozeni minuleho stavu EN2A
		if (C_CHECKBIT(EN2A)) C_SETBIT(PEN2A);
			else C_CLEARBIT(PEN2A);
			
		// ulozeni minuleho stavu EN2B
		if (C_CHECKBIT(EN2B)) C_SETBIT(PEN2B);
			else C_CLEARBIT(PEN2B);
			
			
			
		// vzestupna hrana - EN1A
		if ((C_CHECKBIT(EN1A)) && (!C_CHECKBIT(PEN1A))) {
			if (C_CHECKBIT(EN1B)) t_en1++;
				else t_en1--;
		} else
		// sestupna hrana - EN1A
		if ((!C_CHECKBIT(EN1A)) && (C_CHECKBIT(PEN1A))) {
			if (!C_CHECKBIT(EN1B)) t_en1++;
				else t_en1--;
		}
		
		
		
		
		// vzestupna hrana - EN1B
		if ((C_CHECKBIT(EN1B)) && (!C_CHECKBIT(PEN1B))) {
			if (!C_CHECKBIT(EN1A)) t_en1++;
				else t_en1--;
		} else
		// sestupna hrana - EN1B
		if ((!C_CHECKBIT(EN1B)) && (C_CHECKBIT(PEN1B))) {
			if (C_CHECKBIT(EN1A)) t_en1++;
				else t_en1--;
		}
		
		
		// ulozeni minuleho stavu EN1A
		if (C_CHECKBIT(EN1A)) C_SETBIT(PEN1A);
			else C_CLEARBIT(PEN1A);
			
		// ulozeni minuleho stavu EN1B
		if (C_CHECKBIT(EN1B)) C_SETBIT(PEN1B);
			else C_CLEARBIT(PEN1B);

}


int main(void) {

	
	

	ioinit();
	
	
	
	C_CLEARBIT(INPUT1);
	C_CLEARBIT(INPUT2);
	
	uint8_t pocit=0;
	volatile unsigned char buffer[10]="";

	while(1) {
	
		/*if (C_CHECKBIT(EN2A)) C_SETBIT(LED);
			else C_CLEARBIT(LED);
		_delay_ms(50);
		*/
		
		
		switch (uart_getc()) {
		
		
			case '+': speed++; break;
			case '-': speed--; break;
			case '0': speed=0; break;
			case '*': speed*=-1; break;
		
		
		}
		
		
		if (++pocit==10) {
		
			
			pocit = 0;
	
			uart_puts_P("en2 tic: ");
			uart_puts(itoa(t_en2,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("en2 dif: ");
			uart_puts(itoa(diff2,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("OCR1B: ");
			uart_puts(itoa(OCR1B,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("e: ");
			uart_puts(itoa(e,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("sum: ");
			uart_puts(itoa(sum,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("act: ");
			uart_puts(itoa(act,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			
			uart_puts_P("speed: ");
			uart_puts(itoa(speed,buffer,10));
			uart_puts_P("\n");
			uart_putc(13);
			uart_puts_P("\n");
			uart_putc(13);
		
		}
		
		_delay_ms(100);
		

	
	
	}


}