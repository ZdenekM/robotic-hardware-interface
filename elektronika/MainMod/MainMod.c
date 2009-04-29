// MainMod - k�d pro ��dic� modul
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: ujet� zadan� vzd�lenosti
// TODO: pln� skenov�n� okol�
// TODO: n�hodn� proj��ka
// TODO: zastaven� p�i v�padku SensMod
// TODO: poladit komunikaci s PC - ne�ekat ve smy�ce

#include "MainMod.h"

// *** GLOBALNI PROMENNE **************************************************
// stavov� prom�nn� modulu
tmod_state mod_state;

// stav komunikace - rs485
tcomm_state comm_state;

// stav komunikace - rs232
tcomm_state pccomm_state;

// strukt. pro regul�tor ujet� vzd�lenosti
tdist_reg dist_reg;

// reg. pro ot��en�
tangle_reg angle_reg;

// struktura pro stav motor�
tmotors motors;

// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

// stav senzor�
tsens sens;

// �asov� flagy
#define F_1HZ flags, 0
#define F_50HZ flags, 1



// USART0 - komunikace s PC
ISR(USART0_RX_vect) {

	if (CHECKBIT(UCSR0A,FE0)) pccomm_state.frame_error++;
	receivePacket(UDR0,&pccomm_state);

}

// USART0 - UDR Empty
ISR(USART0_UDRE_vect) {

	sendPacket(&UDR0,&pccomm_state);

	if (pccomm_state.send_state==PS_READY) CLEARBIT(UCSR0B,UDRIE0);

}

// USART1 - Rx Complete
ISR(USART1_RX_vect) {

	if (CHECKBIT(UCSR1A,FE1)) comm_state.frame_error++;
	receivePacket(UDR1,&comm_state);

}


// USART1 - UDR Empty
ISR(USART1_UDRE_vect) {

		// po odvys�l�n� adresy zru�it nastaven� 9. bit
		if (comm_state.send_state>=PS_ADDR) CLEARBIT(UCSR1B,TXB81);
		else SETBIT(UCSR1B,TXB81);

		sendPacket(&UDR1,&comm_state);

		if (comm_state.send_state==PS_READY) CLEARBIT(UCSR1B,UDRIE1);


}



// p�eru�en� - 50 Hz
ISR(TIMER0_COMP_vect) {


	// po��tadlo pro obnoven� lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	C_SETBIT(F_50HZ);

	// aktualizace vnitrniho casu
	updateTime(&mod_state);

	// kontrola tla��tek
	checkButtons(&mod_state);

	// po��tadlo timeoutu pro p��jem po RS485
	receiveTimeout(&comm_state);

	// po��tadlo timeoutu pro p��jem po RS232
	receiveTimeout(&pccomm_state);

	// 1 Hz
	if (++lcdc==50) {

		lcdc=0;

		// zv��en� po��tadla pro timeout komunikace s PC
		if (mod_state.pc_comm_to<PcCommTo) mod_state.pc_comm_to++;

		// nastaven� p��znak�
		C_SETBIT(F_1HZ);



	}



}




int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	mod_state.menu_state = M_STANDBY;

	// nastaven� zdroje ��zen� na PC
	mod_state.control = C_PC;

	//pccomm_state.receive_state = PR_WAITING;

	// zji�t�n� PID konstant nastaven�ch v EEPROM
	getMotorPID(10);


	// nekone�n� smy�ka
    while (1) {

    	// reset watchdogu
    	wdt_reset();


    	// obsluha LCD
    	if (C_CHECKBIT(F_1HZ)) {

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

        	if (CHECKBIT(mod_state.buttons,ABUTT3)) {

        		if (++mod_state.control>C_PC) mod_state.control = C_AUTO;

        	    CLEARBIT(mod_state.buttons,ABUTT3);

        	}

        	if (CHECKBIT(mod_state.buttons,ABUTT4)) {

        		//  nekone�n� smy�ka -> reset modulu
        		while(1);

        	}

    		C_CLEARBIT(F_1HZ);
    		manageLcd();


    	}


    	// obsluha periferi� a pod��zen�ch modul� ---------------------------------------------------------------------------

    	if (C_CHECKBIT(F_50HZ)) {

    		C_CLEARBIT(F_50HZ);

			update_joystick();

			// �ten� stavu lev�ch motor�
			getMotorInfo(10,&motors.m[FRONT_LEFT],&motors.m[REAR_LEFT]);

			// �ten� stavu prav�ch motor�
			getMotorInfo(11,&motors.m[FRONT_RIGHT],&motors.m[REAR_RIGHT]);

			// na�ten� dat ze senzor�
			getFastSensorState();

			// regul�tor pro ujetou vzd�lenost
			distReg();

			// regul�tor oto�en�
			angleReg();



		switch(mod_state.control) {

		// zdroj ��zen� nastaven na PC
		case C_PC: {

			// zastaven� p�i timeoutu komunikace s PC - 5s
			if (mod_state.pc_comm_to >= PcCommTo)
				// po�. rychlost nen� nula - zastavit
				setMotorsSpeed(0,0);


		} break;

		// zdroj ��zen� nastaven na joystick
		case C_JOY: {

			joyRide();


		} break;

		// autonomn� operace
		case C_AUTO: {

			// TODO: n�hodn� proj��ka

			/*if (dist_reg.state == R_READY) {

				if (sens.us_fast>200) setDistReg(&dist_reg,sens.us_fast-200);

			} else {

				if (sens.us_fast > (dist_reg.req_dist+200)) setDistReg(&dist_reg,sens.us_fast-200);

			}*/

			// test oto�en� o zadan� �hel
			static uint8_t flag = 0;

			if (flag==0) {

				flag = 1;

				setAngleReg(90);



			}

		} break;




		} //switch


    	} // 50 Hz



    	// obsluha komunikace s PC ---------------------------------------------------------------------------


		//pccomm_state.receive_state = PR_WAITING;

		//while (pccomm_state.receive_state!=PR_PACKET_RECEIVED && pccomm_state.receive_state!=PR_TIMEOUT && pccomm_state.receive_state!=PR_READY);

    	// crc souhlas� -> �sp�n� p�ijet� paketu
    	if ((pccomm_state.receive_state==PR_PACKET_RECEIVED) && checkPacket(&pccomm_state)) {

    		// vynulov�n� po��tadla timeoutu kom. s PC
    		mod_state.pc_comm_to = 0;

    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zp�t stejn� paket
    		case P_ECHO: {

    			// vytvo�en� ECHO paketu
    			makePacket(&pccomm_state.op,pccomm_state.ip.data,pccomm_state.ip.len,P_ECHO,0);

    			// obsluha odesl�n� paketu
    			sendPCPacketE(&pccomm_state);


    		} break;

    		// voln� j�zda - ovl�d�n� joystickem
    		case PC_FREE_RIDE: {

    			int16_t spl, spr;

    			if (mod_state.control == C_PC ) {

					// rychlost pro lev� motory
					spl = pccomm_state.ip.data[0];
					spl |= pccomm_state.ip.data[1]<<8;

					// rychlost pro prav� motory
					spr = pccomm_state.ip.data[2];
					spr |= pccomm_state.ip.data[3]<<8;

					setMotorsSpeed(spl,spr);


    			}

    		} break;

    		// po�adavek na nastaven� PID konstant
    		case P_MOTOR_SETPID: {

    			motors.P = pccomm_state.ip.data[0];
    			motors.I = pccomm_state.ip.data[1];
    			motors.D = pccomm_state.ip.data[2];

    			setMotorPID(10);
    			setMotorPID(11);


    		} break;

    		// po�adavek na p�e�ten� PID konstant
    		case P_MOTOR_GETPID: {

    			uint8_t arr[3];

    			arr[0] = motors.P;
    		    arr[1] = motors.I;
    		    arr[2] = motors.D;

    		    makePacket(&pccomm_state.op,arr,3,P_MOTOR_GETPID,0);

    		    // obsluha odesl�n� paketu
    		    sendPCPacketE(&pccomm_state);


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

				aspeedl = (motors.m[FRONT_LEFT].act_speed + motors.m[REAR_LEFT].act_speed)/2;
				rspeedl = (motors.m[FRONT_LEFT].req_speed + motors.m[REAR_LEFT].req_speed)/2;
				distl = (motors.m[FRONT_LEFT].distance + motors.m[REAR_LEFT].distance)/2;

				aspeedr = (motors.m[FRONT_RIGHT].act_speed + motors.m[REAR_RIGHT].act_speed)/2;
				rspeedr = (motors.m[FRONT_RIGHT].req_speed + motors.m[REAR_RIGHT].req_speed)/2;
				distr = (motors.m[FRONT_RIGHT].distance + motors.m[REAR_RIGHT].distance)/2;

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

				// obsluha odesl�n� paketu
				sendPCPacketE();


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

    			// obsluha odesl�n� paketu
    			sendPCPacketE();

    		} break;

    		// pln� m��en� - pouze kdy� se stoj�
    		case P_SENS_FULL: {

    			// provede se pouze kdy� robot stoj� a zdroj ��zen� je nastaven� na PC
    			if ((motors.m[FRONT_LEFT].act_speed == 0) && (motors.m[FRONT_RIGHT].act_speed == 0) && (mod_state.control == C_PC)) {

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

    				// obsluha odesl�n� paketu
    				sendPCPacketE();

    			}



    		} break;


    		} // switch

    		//pccomm_state.receive_state = PR_WAITING;

    	} // if


    	//if ((pccomm_state.receive_state == PR_TIMEOUT) || (pccomm_state.receive_state == PR_READY)) pccomm_state.receive_state = PR_WAITING;



    	//_delay_ms(100);

    } // while

    return 0;
}
