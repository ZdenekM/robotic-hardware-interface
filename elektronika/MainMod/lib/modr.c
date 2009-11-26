// MainMod - kód pro řídicí modul
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface
// modr.c -> spec. funkce modulu

#include "modr.h"

extern tmod_state mod_state;
extern tcomm_state comm_state;
extern tcomm_state pccomm_state;
extern tdist_reg dist_reg;
extern tangle_reg angle_reg;
extern tmotors motors;
extern tsens sens;
extern tpower_state power_state;
extern trand_ride rand_ride;


// inicializace regulátoru ujeté vzd.
void initDistReg() {


	dist_reg.lstart_dist = 0;
	dist_reg.req_dist = 0;
	dist_reg.rstart_dist = 0;
	dist_reg.state = R_READY;
	dist_reg.P = 60;

}

void initAngleReg() {

	angle_reg.P = 1;
	//angle_reg.I = 0;
	//angle_reg.D = 0;
	//angle_reg.last_angle = 0;
	angle_reg.req_angle = 0;
	angle_reg.start_angle = 0;
	angle_reg.state = R_READY;
	//angle_reg.sum = 0;


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

	// PORTD5-7 tlačítka, aktivované pullupy, PD3 = TXD1, PD4 = RE/DE
	DDRD = (0<<DDD5)|(0<<DDD6)|(0<<DDD7)|(1<<DDD3)|(1<<DDD4);
	PORTD = (1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7)|(1<<PORTD3)|(1<<PORTD4);

	C_CLEARBIT(LCD_BL);

	lcd_init(LCD_DISP_ON);
	lcd_home();

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

	// nastavení obou UARTů
		// nastaveni uart1 - komunikace s moduly

			// UCSZxx = 111 -> 9bit
			// UCSZxx = 011 -> 8bit
			// RXCIEn: RX Complete Interrupt Enable
			// TXCIEn: TX Complete Interrupt Enable
			// RXENn: Receiver Enable
			// TXENn: Transmitter Enable
			// UCSZn1:0: Character Size
			// usbs - počet stop bitů
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
	UBRR1L = 12;
	UBRR1H = 0;

	UCSR1A = (0<<U2X1)|(0<<MPCM1);
	UCSR1B = (1<<UCSZ12)|(0<<UDRIE1)|(1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);
	UCSR1C = (1<<USBS1)|(0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(1<<UCSZ11)|(1<<UCSZ10);

	// nastavení ADC
	// reference = AVCC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1);

	// povolení ADC
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	// inicializace struktury comm_state
	comm_state_init(&comm_state);
	comm_state_init(&pccomm_state);

	// inic. struktur motorů
	motor_init(&motors);

	// inicializace regulátoru ujeté vzd.
	initDistReg();

	initAngleReg();


	if (CHECKBIT(MCUCSR,WDRF)) CLEARBIT(MCUCSR,WDRF);
	else {

		lcd_puts_P("Starting-up...");

		_delay_ms(2000);

		// zjistí dostupnost dalších modulů
		//initModules();


	}

	lcd_clrscr();

	// povolení příjmu
	C_CLEARBIT(RS485_SEND);

	// nastavení watchdogu -> 1s
	//WDTCR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0);

	wdt_reset();

	sei();

}


// volá se z přerušení
// čtení a zpracování stavu tlačítek
void checkButtons () {

// udava kolikrat po sobe musi byt tlacitko sepnute
#define BUTT_DEBOUNCE 5

	// pomocné proměnné pro ošetření zákmitů tlačítek bez čekání
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

// volá se z přerušení
// počítadlo pro čas
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
// vypíše čas na lcd - pravý horní roh
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

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("B1:%7u"),power_state.ubatt1);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("B2:%7u"),power_state.ubatt2);
	lcd_puts(abuff);

	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("MD:%7u"),power_state.umod);
	lcd_puts(abuff);

	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("MT:%7u"),power_state.umot);
	lcd_puts(abuff);


}

// nastaví offset joysticku
void setJoyZero() {

	/*mod_state.joy_x_offset = mod_state.joy_x - 512;
	mod_state.joy_y_offset = mod_state.joy_y - 512;*/

}

// nastavení obou UARTů
void set_uarts() {




}



void set_adc() {




}

// spustí AD převod pro určení vychýlení joysticku
void update_joystick() {

	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitů
	ADMUX |= 6&0x3F; // osa Y

	// spustí převod
	SETBIT(ADCSRA,ADSC);

	// čeká na dokončení převodu
	while (CHECKBIT(ADCSRA,ADSC ));

	mod_state.joy_y = ((mod_state.joy_y + ADCW) / 2) - mod_state.joy_y_offset; // filtr

	// nastavení kanálu
	ADMUX &= 0xC0; // vynulování 5 spodnich bitů
	ADMUX |= 7&0x3F; // osa X

	// spustí převod
	SETBIT(ADCSRA,ADSC);

	// čeká na dokončení převodu
	while (CHECKBIT(ADCSRA,ADSC ));

	mod_state.joy_x = ((mod_state.joy_x + ADCW) / 2) - mod_state.joy_x_offset; // filtr



}

// regulátor pro ujetí zadané vzdálenosti
void distReg() {

	if (dist_reg.req_dist!=0 && dist_reg.state==R_RUNNING) {

		int32_t lspeed,rspeed;

		int32_t lact_dist = (motors.m[FRONT_LEFT].distance + motors.m[REAR_LEFT].distance)/2;
		int32_t ract_dist = (motors.m[FRONT_RIGHT].distance + motors.m[REAR_RIGHT].distance)/2;

		dist_reg.le = (dist_reg.req_dist - (lact_dist - dist_reg.lstart_dist));
		dist_reg.re = (dist_reg.req_dist - (ract_dist - dist_reg.rstart_dist));

		// výpočet akčního zásahu -> rychlosti
		lspeed = (dist_reg.P)*dist_reg.le;
		rspeed = (dist_reg.P)*dist_reg.re;

		lspeed /= 100;
		rspeed /= 100;

		if (lspeed>250) lspeed = 250;
		else if (lspeed<-250) lspeed = -250;

		if (rspeed>250) rspeed = 250;
		else if (rspeed<-250) rspeed = -250;


		// hotovo, odchylka je pod požadovanou mezí
		if (labs(dist_reg.le) < 10 && labs(dist_reg.re) < 10) {

			dist_reg.state = R_READY;
			setMotorsSpeed(0,0);
		}


		// pokud senzory detekují překážku - nedá se jet dál, hotovo
		else if (!setMotorsSpeed(lspeed,rspeed)) {

			setMotorsSpeed(0,0);
			dist_reg.state = R_OBST;

		}


	}


}

// zobrazí info o distreg na lcd
void distRegInfo() {

	char abuff[11];


	// levá počáteční vzdálenost
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("LS:%7d"),dist_reg.lstart_dist);
	lcd_puts(abuff);

	// levá odchylka
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("LE:%7d"),dist_reg.le);
	lcd_puts(abuff);

	// žádaná vzdálenost
	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("RD:%7d"),dist_reg.req_dist);
	lcd_puts(abuff);

	// pravá počáteční vzdálenost
	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("RS:%7d"),dist_reg.rstart_dist);
	lcd_puts(abuff);

	// levá odchylka
	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("RE:%7d"),dist_reg.re);
	lcd_puts(abuff);

	// stav regulátoru
	lcd_gotoxy(10,3);
	if (dist_reg.state==R_READY) sprintf_P(abuff,PSTR("ST:  READY")); // připraven
	else if (dist_reg.state==R_RUNNING) sprintf_P(abuff,PSTR("ST:    RUN")); // běží
	else sprintf_P(abuff,PSTR("ST:   OBST")); // překážka

	lcd_puts(abuff);


}

// zobrazí info o anglereg na lcd
void angleRegInfo() {

	char abuff[11];


	// počáteční azimut
	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("SA:%7d"),angle_reg.start_angle);
	lcd_puts(abuff);

	// žádané otočení
	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("RA:%7d"),angle_reg.req_angle);
	lcd_puts(abuff);

	// aktuální azimut
	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("AA:%7d"),sens.comp);
	lcd_puts(abuff);

	// regulační odchylka
	lcd_gotoxy(10,1);
	sprintf_P(abuff,PSTR("E :%7d"),angle_reg.e);
	lcd_puts(abuff);

	// rychlost otáčení
	lcd_gotoxy(10,2);
	sprintf_P(abuff,PSTR("SP:%7d"),angle_reg.speed);
	lcd_puts(abuff);


	// stav regulátoru
	lcd_gotoxy(10,3);
	if (angle_reg.state==R_READY) sprintf_P(abuff,PSTR("ST:  READY")); // připraven
	else if (angle_reg.state==R_RUNNING) sprintf_P(abuff,PSTR("ST:    RUN")); // běží
	lcd_puts(abuff);


}

// regulátor pro otočení o zadaný úhel
void angleReg() {

	if (angle_reg.state == R_RUNNING) {

		int16_t comp;

		// výpočet reg. odchylky

		// otáčení doprava
		if (angle_reg.req_angle>0) {

				// deseti se dělí kvůli snížení přesnosti
				if ((sens.comp/10) < (angle_reg.start_angle/10)) comp = sens.comp + 3600;
				else comp = sens.comp;

			}

		// otáčení doleva
		else {

				if ((sens.comp/10) > (angle_reg.start_angle/10)) comp = sens.comp - 3600;
				else comp = sens.comp;

		}

		// výpočet odchylky
		angle_reg.e = (angle_reg.start_angle + angle_reg.req_angle) - comp;

		// výpočet  rychlosti
		angle_reg.speed = labs(2*angle_reg.e);

		angle_reg.last_e = angle_reg.e;

		// omezení rozsahu
		if (angle_reg.speed>100) angle_reg.speed = 100;
		else if (angle_reg.speed<-100) angle_reg.speed = -100;

		// hotovo (odchylka je 0.5°)
		if (labs(angle_reg.e)<5) {

			angle_reg.speed = 0;
			angle_reg.state = R_READY;

		}

		// určení směru otáčení
		if (angle_reg.e>0) setMotorsSpeed(angle_reg.speed,-angle_reg.speed);
		else setMotorsSpeed(-angle_reg.speed,angle_reg.speed);

		// vynulovat integrátory (zamezení nechtěného otáčení, šetření baterií)
		if (angle_reg.state == R_READY) clearRegSum();



	} // if



}

void setAngleReg(int16_t angle) {

	if (angle>=-360 && angle<=360 && angle_reg.state==R_READY && dist_reg.state!=R_RUNNING) {

		setMotorsSpeed(0,0);
		angle_reg.req_angle = angle*10;
		angle_reg.start_angle = sens.comp;
		angle_reg.state = R_RUNNING;

	}

}

// nastavení regulátoru ujeté vzd.
void setDistReg(int16_t dist) {

	// jenom pokud neběží reg. otočení
	if (angle_reg.state==R_READY) {

		// nastavení počáteční vzdálenosti
		dist_reg.lstart_dist = (motors.m[FRONT_LEFT].distance + motors.m[REAR_LEFT].distance)/2;
		dist_reg.rstart_dist = (motors.m[FRONT_RIGHT].distance + motors.m[REAR_RIGHT].distance)/2;

		// zadání požadované vzdálenosti k ujetí
		dist_reg.req_dist = dist;

		dist_reg.state = R_RUNNING;

	}

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

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("PCCommStat");
	    	commStat(&pccomm_state);

	    } break;

	    // levý přední motor
	    case M_MLF: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftFront");
	    	motStat(&motors.m[FRONT_LEFT]);

	    } break;

	    // levý zadní motor
	    case M_MLR: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MLeftRear");
	    	motStat(&motors.m[REAR_LEFT]);


	    } break;


	    // pravý přední motor
	    case M_MRF: {

	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("MRightFront");
	    	motStat(&motors.m[FRONT_RIGHT]);

	    } break;

	    // pravý zadní motor
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

	    case M_ANGLEREG: {


	    	writeTime();
	    	lcd_gotoxy(0,0);
	    	lcd_puts_P("AngleReg");
	    	angleRegInfo();



	    } break;

	    // zobrazení stavu joysticku
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


	// jedeme dopředu
	if (sp > 10) {

		// zatáčení doprava
		if (ot<-10) setMotorsSpeed(sp,sp+(ot/2));

		// zatáčení doleva
		else if (ot > 10) setMotorsSpeed(sp-(ot/2),sp);

		// jedeme rovně
		else setMotorsSpeed(sp,sp);

	// jedeme dozadu
	} else if (sp < -10) {

		// zatáčení doprava
		if (ot<-10) setMotorsSpeed(sp,sp-(ot/2));

		// zatáčení doleva
		else if (ot > 10) setMotorsSpeed(sp+(ot/2),sp);

		// jedeme rovně
		else setMotorsSpeed(sp,sp);


	// otáčení na místě - doprava
	} else if (ot<-10){

		setMotorsSpeed(-ot,ot);

	// otáčení na místě - doleva
	} else if (ot>10) {

		setMotorsSpeed(-ot,ot);

	// zastavení*/
	} else {

		setMotorsSpeed(0,0);

		}




}

void randomRide() {

	switch(rand_ride.state) {

	// jízda rovně
	case STRAIGHT: {

		if ((sens.us_fast==0 || sens.us_fast>200)
			&& (sens.sharp[0]==0 || sens.sharp[0]>200)
			&& (sens.sharp[1]==0 || sens.sharp[1]>200)) {

				setMotorsSpeed(250,250);
				rand_ride.obsc = 0;

		}
		else {

			if (++rand_ride.obsc==5) {

				setMotorsSpeed(0,0);
				rand_ride.state = BRAKING;

				rand_ride.obsc = 0;

			}



		}

	} break;

	// čekání na zastavení
	case BRAKING: {

		if (motors.m[FRONT_LEFT].act_speed==0 && motors.m[FRONT_RIGHT].act_speed==0) {

			while(comm_state.send_state != PS_READY);

			// příkaz pro plné skenování
			makePacket(&comm_state.op,NULL,0,P_SENS_FULL,21);
			sendPacketE();

			rand_ride.state = WFFS;


		}

	}

	// čekání na plný sken
	case WFFS: {

		if (sens.new_full_flag==1) {

			// vlevo je víc místa než rovně
			if (sens.us_full[0] > sens.us_full[2]) {

				// otáčení doleva
				setMotorsSpeed(-50,50);
				rand_ride.dist = sens.us_full[0];
				rand_ride.state = TURNING;

			} else

			// vpravo je víc místa než rovně
			if (sens.us_full[4] > sens.us_full[2]) {

				// otáčení doprava
				setMotorsSpeed(50,-50);
				rand_ride.dist = sens.us_full[4];
				rand_ride.state = TURNING;


			// TODO: dodělat couvání, pokud to nejde rovně
			} else { // nejvíc místa je rovně


				rand_ride.state = STRAIGHT;


			}



			sens.new_full_flag = 0;

		}



	} break;

	// otáčení, dokud nebude před robotem dost místa
	case TURNING: {

		if (sens.us_fast >=rand_ride.dist) {

				setMotorsSpeed(0,0);
				clearRegSum();
				rand_ride.turn_to = 0;
				rand_ride.state = STRAIGHT;

		} else {

			if (++rand_ride.turn_to==50) {

				// nedaří se najít vybranou vzdálenost, zkusíme polovinu :)
				rand_ride.dist /= 2;
				rand_ride.turn_to = 0;

				}


		}



	} break;


	} // switch auto;


}
