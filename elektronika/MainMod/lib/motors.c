
#include "motors.h"

extern tmod_state mod_state;
extern tcomm_state comm_state;
extern tcomm_state pccomm_state;
extern tdist_reg dist_reg;
extern tangle_reg angle_reg;
extern tmotors motors;
extern tsens sens;

// inicializace struktur
void motor_init() {

	for (uint8_t i=0;i<4;i++) {

		motors.m[i].act_speed = 0;
		motors.m[i].current = 0;
		motors.m[i].distance = 0;
		motors.m[i].req_speed = 0;
		motors.m[i].state = 0;
		motors.m[i].temp = 0;
		motors.m[i].load = 0;


	}



}

// volá se z main
// na LCD zobrazí info z motoru
void motStat(tmotor *m) {

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

// volá se z main
// dekóduje pøijaté info z modulu MotorControl
void decodeMotorInfo(tmotor *mf, tmotor *mr) {

		// údaje pro pøední motor ----------------------------------

		// požadovaná rychlost
		mf->req_speed = comm_state.ip.data[0];
		mf->req_speed |= comm_state.ip.data[1]<<8;

		// aktuální rychlost
		mf->act_speed = comm_state.ip.data[2];
		mf->act_speed |= comm_state.ip.data[3]<<8;

		// stav motoru
		mf->state = comm_state.ip.data[4];

		// proud motoru
		mf->current = comm_state.ip.data[5];

		// teplota motoru
		mf->temp = comm_state.ip.data[6];

		// ujetá vzdálenost
		mf->distance = comm_state.ip.data[7];
		mf->distance  |= comm_state.ip.data[8]<<8;
		mf->distance  |= comm_state.ip.data[9]<<16;
		mf->distance  |= comm_state.ip.data[10]<<24;

		// výkon motoru
		mf->load = comm_state.ip.data[11];


		// údaje pro zadní motor -------------------------------------------------

		// požadovaná rychlost
		mr->req_speed = comm_state.ip.data[0];
		mr->req_speed |= comm_state.ip.data[1]<<8;

		// aktuální rychlost
		mr->act_speed = comm_state.ip.data[12];
		mr->act_speed |= comm_state.ip.data[13]<<8;

		// stav motoru
		mr->state = comm_state.ip.data[14];

		// proud motoru
		mr->current = comm_state.ip.data[15];

		// teplota motoru
		mr->temp = comm_state.ip.data[16];

		// ujetá vzdálenost
		mr->distance = comm_state.ip.data[17];
		mr->distance  |= comm_state.ip.data[18]<<8;
		mr->distance  |= comm_state.ip.data[19]<<16;
		mr->distance  |= comm_state.ip.data[20]<<24;

		// výkon motoru
		mr->load = comm_state.ip.data[21];

}


// zobrazení PID konstant na lcd
void motPID() {

	char abuff[11];

	lcd_gotoxy(0,1);
	sprintf_P(abuff,PSTR("P:%7u"),motors.P);
	lcd_puts(abuff);

	lcd_gotoxy(0,2);
	sprintf_P(abuff,PSTR("I:%7u"),motors.I);
	lcd_puts(abuff);

	lcd_gotoxy(0,3);
	sprintf_P(abuff,PSTR("D:%7u"),motors.D);
	lcd_puts(abuff);

}


// naète z modulu info o motorech a vyplní ho do struktur tmotor
void getMotorInfo(uint8_t addr, tmotor *front, tmotor *rear) {

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


// nastavení požadované rychlosti motorù
void setMotorSpeed(uint8_t addr, int16_t speed) {

	uint8_t sarr[2];

	sarr[0] = speed;
	sarr[1] = speed>>8;

	makePacket(&comm_state.op,sarr,2,P_MOTOR_COMM,addr);

	sendPacketE();


}

// nastavení rychlosti motorù -> v závislosti na vzdálenosti pøekážky
void setMotorsSpeed(int16_t left, int16_t right) {

	// omezení rozsahu
	if (left>250) left=250;
	if (left<-250) left=-250;

	if (right>250) right=250;
	if (right<-250) right=-250;


	// jedeme dopøedu
	if ((left > 0) && (right > 0)) {

		int16_t lowest = 500;

		if (sens.us_fast!=0 && sens.us_fast<lowest) lowest = sens.us_fast;
		if (sens.sharp[0]!=0 && sens.sharp[0]<lowest) lowest = sens.sharp[0];
		if (sens.sharp[1]!=0 && sens.sharp[1]<lowest) lowest = sens.sharp[1];

		// nebezpeèná vzdálenost - nutno omezit rychlost
		if (lowest<350) {

			if (left>(lowest-100)) left = lowest-100;
			if (right>(lowest-100)) right = lowest-100;

		}

	// jedeme dozadu
	} else if (left<0 && right < 0) {

		int16_t lowest= 350;

		if (sens.sharp[2]!=0 && sens.sharp[2]<lowest) lowest = sens.sharp[2];
		if (sens.sharp[3]!=0 && sens.sharp[3]<lowest) lowest = sens.sharp[3];

		// nebezpeèná vzdálenost - nutno omezit rychlost
		if (lowest<350) {

			if (abs(left)>(lowest-100)) left = -(lowest-100);
			if (abs(right)>(lowest-100)) right = -(lowest-100);

				}

	} // else -> kontrola pøi otáèení??


	// zmìna rychlosti - posílá se pouze pokus se liší nová a pùvodní hodnota
	if (motors.m[FRONT_LEFT].req_speed!=left) setMotorSpeed(10,left);
	if (motors.m[FRONT_RIGHT].req_speed!=right)	setMotorSpeed(11,right);


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

		motors.P = comm_state.ip.data[0];
		motors.I = comm_state.ip.data[1];
		motors.D = comm_state.ip.data[2];

	}

	comm_state.receive_state = PR_READY;


}

// nastavení PID parametrù
void setMotorPID(uint8_t addr) {

	uint8_t sarr[3];

	sarr[0] = motors.P;
	sarr[1] = motors.I;
	sarr[2] = motors.D;

	makePacket(&comm_state.op,sarr,3,P_MOTOR_SETPID,addr);

	sendPacketE();


}
