// MainMod - k�d pro ��dic� modul
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface
// modr.c -> spec. funkce modulu

#include "modr.h"

extern tmod_state mod_state;
extern tcomm_state comm_state;
extern tcomm_state pccomm_state;
extern tdist_reg dist_reg;
extern tangle_reg angle_reg;
extern tmotors motors;
extern tsens sens;


// inicializace regul�toru ujet� vzd.
void initDistReg() {


	dist_reg.lstart_dist = 0;
	dist_reg.req_dist = 0;
	dist_reg.rstart_dist = 0;
	dist_reg.state = R_READY;
	dist_reg.P = 60;

}

void initAngleReg() {

	angle_reg.P = 30;
	angle_reg.I = 0;
	angle_reg.D = 0;
	angle_reg.last_angle = 0;
	angle_reg.req_angle = 0;
	angle_reg.start_angle = 0;
	angle_reg.state = R_READY;
	angle_reg.sum = 0;


}

// inicializace
void ioinit() {

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
	UCSR0B = (0<<UCSZ02)|(0<<UDRIE0)|(1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (0<<USBS0)|(0<<UMSEL0)|(0<<UPM01)|(0<<UPM00)|(1<<UCSZ01)|(1<<UCSZ00);

	// RS485, 9n2
	// 9600bd - UBRR1L = 103;
	//UBRR1L = 68; // 14400
	//UBRR1L = 25; // 38400
	//UBRR1L = 12; // 76800
	UBRR1L = 12; // 250000
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(0<<UDRIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);

	// nastaven� ADC
	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolen� ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	// inicializace struktury comm_state
	comm_state_init(&comm_state);
	comm_state_init(&pccomm_state);

	// inic. struktur motor�
	motor_init(&motors);

	// inicializace regul�toru ujet� vzd.
	initDistReg();

	initAngleReg();


	if (CHECKBIT(MCUCSR,WDRF)) CLEARBIT(MCUCSR,WDRF);
	else {

		lcd_puts_P("Starting-up...");

		_delay_ms(2000);

		// zjist� dostupnost dal��ch modul�
		//initModules();


	}

	lcd_clrscr();

	// povolen� p��jmu
	C_CLEARBIT(RS485_SEND);

	// nastaven� watchdogu -> 1s
	//WDTCR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0);

	wdt_reset();

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

// nastaven� obou UART�
void set_uarts() {




}



void set_adc() {




}

// spust� AD p�evod pro ur�en� vych�len� joysticku
void update_joystick() {

	// nastaven� kan�lu
	ADMUX &= 0xC0; // vynulov�n� 5 spodnich bit�
	ADMUX |= 6&0x3F; // osa Y

	// spust� p�evod
	SETBIT(ADCSRA,ADSC);

	// �ek� na dokon�en� p�evodu
	while (CHECKBIT(ADCSRA,ADSC ));

	mod_state.joy_y = (mod_state.joy_y + ADCW) / 2; // filtr

	// nastaven� kan�lu
	ADMUX &= 0xC0; // vynulov�n� 5 spodnich bit�
	ADMUX |= 7&0x3F; // osa X

	// spust� p�evod
	SETBIT(ADCSRA,ADSC);

	// �ek� na dokon�en� p�evodu
	while (CHECKBIT(ADCSRA,ADSC ));

	mod_state.joy_x = (mod_state.joy_x + ADCW) / 2; // filtr



}

// regul�tor pro ujet� zadan� vzd�lenosti
void distReg() {

	if (dist_reg.req_dist!=0 && dist_reg.state==R_RUNNING) {

		int32_t lspeed,rspeed;

		int32_t lact_dist = (motors.m[FRONT_LEFT].distance + motors.m[REAR_LEFT].distance)/2;
		int32_t ract_dist = (motors.m[FRONT_RIGHT].distance + motors.m[REAR_RIGHT].distance)/2;

		dist_reg.le = (dist_reg.req_dist - (lact_dist - dist_reg.lstart_dist));
		dist_reg.re = (dist_reg.req_dist - (ract_dist - dist_reg.rstart_dist));

		// v�po�et ak�n�ho z�sahu -> rychlosti
		lspeed = (dist_reg.P)*dist_reg.le;
		rspeed = (dist_reg.P)*dist_reg.re;

		lspeed /= 100;
		rspeed /= 100;

		if (lspeed>250) lspeed = 250;
		else if (lspeed<-250) lspeed = -250;

		if (rspeed>250) rspeed = 250;
		else if (rspeed<-250) rspeed = -250;


		// hotovo, odchylka je pod po�adovanou mez�
		if (labs(dist_reg.le) < 10 && labs(dist_reg.re) < 10) {

			dist_reg.state = R_READY;
			setMotorsSpeed(0,0);
		}


		// pokud senzory detekuj� p�ek�ku - ned� se jet d�l, hotovo
		else if (!setMotorsSpeed(lspeed,rspeed)) {

			setMotorsSpeed(0,0);
			dist_reg.state = R_OBST;

		}


	}


}

// zobraz� info o distreg na lcd
void distRegInfo() {

	char abuff[11];


	// lev� po��te�n� vzd�lenost
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("LS:%7d"),dist_reg.lstart_dist);
	lcd_puts(abuff);

	// lev� odchylka
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("LE:%7d"),dist_reg.le);
	lcd_puts(abuff);

	// ��dan� vzd�lenost
	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("RD:%7d"),dist_reg.req_dist);
	lcd_puts(abuff);

	// prav� po��te�n� vzd�lenost
	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("RS:%7d"),dist_reg.rstart_dist);
	lcd_puts(abuff);

	// lev� odchylka
	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("RE:%7d"),dist_reg.re);
	lcd_puts(abuff);

	// stav regul�toru
	lcd_gotoxy(10,3);
	if (dist_reg.state==R_READY) sprintf_P(abuff,PSTR("ST:  READY")); // p�ipraven
	else if (dist_reg.state==R_RUNNING) sprintf_P(abuff,PSTR("ST:    RUN")); // b��
	else sprintf_P(abuff,PSTR("ST:   OBST")); // p�ek�ka

	lcd_puts(abuff);


}

//TODO: nefunguje - OPRAVIT
// regul�tor pro oto�en� o zadan� �hel
void angleReg() {

	if (angle_reg.state == R_RUNNING) {

		int16_t speed,e;

		// v�po�et odchylky (relativn�, ne absolutn�)
		e = angle_reg.req_angle - (sens.comp - angle_reg.start_angle);

		// v�po�et  rychlosti
		speed = angle_reg.P*e;

		speed /= 100;

		// omezen� rozsahu
		if (speed>250) speed = 250;
		else if (speed<-250) speed = -250;

		// hotovo (odchylka je 0.5�)
		if (labs(e)<5) {

			speed = 0;
			angle_reg.state = R_READY;

		}

		// ur�en� sm�ru ot��en�
		if (e>0) setMotorsSpeed(speed,-speed);
		else setMotorsSpeed(-speed,speed);



	}



}

void setAngleReg(int16_t angle) {

	if (angle>=-360 && angle<=360 && angle_reg.state==R_READY) {

		setMotorsSpeed(0,0);
		angle_reg.req_angle = angle*10;
		angle_reg.start_angle = sens.comp;
		angle_reg.state = R_RUNNING;

	}

}

// nastaven� regul�toru ujet� vzd.
void setDistReg(int16_t dist) {

	// nastaven� po��te�n� vzd�lenosti
	dist_reg.lstart_dist = (motors.m[FRONT_LEFT].distance + motors.m[REAR_LEFT].distance)/2;
	dist_reg.rstart_dist = (motors.m[FRONT_RIGHT].distance + motors.m[REAR_RIGHT].distance)/2;

	// zad�n� po�adovan� vzd�lenosti k ujet�
	dist_reg.req_dist = dist;

	dist_reg.state = R_RUNNING;


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

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("PCCommStat");
	    	commStat(&pccomm_state);

	    } break;

	    // lev� p�edn� motor
	    case M_MLF: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftFront");
	    	motStat(&motors.m[FRONT_LEFT]);

	    } break;

	    // lev� zadn� motor
	    case M_MLR: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftRear");
	    	motStat(&motors.m[REAR_LEFT]);


	    } break;


	    // prav� p�edn� motor
	    case M_MRF: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightFront");
	    	motStat(&motors.m[FRONT_RIGHT]);

	    } break;

	    // prav� zadn� motor
	    case M_MRR: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightRear");
	    	motStat(&motors.m[REAR_RIGHT]);


	    } break;

	    case M_PID: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("PIDconst");
	    	motPID();

	    } break;

	    case M_SENS: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("Sensors");
	    	sensInfo();

	    } break;

	    case M_SENS_FULL: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("FSensors");
	    	sensFullInfo();

	    	    } break;

	    case M_DISTREG: {


	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("DistReg");
	    	distRegInfo();



	    } break;

	    // zobrazen� stavu joysticku
	    case M_JOYSTICK: {

	    	writeTime();
	    	joy();

	    } break;


	    		} // switch



}

// obsluha joysticku
void joyRide() {

	int16_t sp = 0, ot = 0;

	sp = (int16_t)(mod_state.joy_y-511)/2;

	ot = (int16_t)(mod_state.joy_x-511)/2; // 255 vlevo, -255 vpravo


	// jedeme dop�edu
	if (sp > 10) {

		// zat��en� doprava
		if (ot<-10) setMotorsSpeed(sp,sp+(ot/2));

		// zat��en� doleva
		else if (ot > 10) setMotorsSpeed(sp-(ot/2),sp);

		// jedeme rovn�
		else setMotorsSpeed(sp,sp);

	// jedeme dozadu
	} else if (sp < -10) {

		// zat��en� doprava
		if (ot<-10) setMotorsSpeed(sp,sp-(ot/2));

		// zat��en� doleva
		else if (ot > 10) setMotorsSpeed(sp+(ot/2),sp);

		// jedeme rovn�
		else setMotorsSpeed(sp,sp);


	// ot��en� na m�st� - doprava
	} else if (ot<-10){

		setMotorsSpeed(-ot,ot);

	// ot��en� na m�st� - doleva
	} else if (ot>10) {

		setMotorsSpeed(-ot,ot);

	// zastaven�*/
	} else {

		setMotorsSpeed(0,0);

		}




}
