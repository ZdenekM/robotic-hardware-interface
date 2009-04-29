// MainMod - kód pro øídicí modul
// autor: Zdenìk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface

// TODO: ujetí zadané vzdálenosti
// TODO: plné skenování okolí
// TODO: náhodná projížïka
// TODO: zastavení pøi výpadku SensMod
// TODO: poladit komunikaci s PC - neèekat ve smyèce

#include "MainMod.h"

// *** GLOBALNI PROMENNE **************************************************
// stavové promìnné modulu
tmod_state mod_state;

// stav komunikace - rs485
tcomm_state comm_state;

// stav komunikace - rs232
tcomm_state pccomm_state;

// strukt. pro regulátor ujeté vzdálenosti
tdist_reg dist_reg;

// reg. pro otáèení
tangle_reg angle_reg;

// struktura pro stav motorù
tmotors motors;

// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

// stav senzorù
tsens sens;

// èasové flagy
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

		// po odvysílání adresy zrušit nastavený 9. bit
		if (comm_state.send_state>=PS_ADDR) CLEARBIT(UCSR1B,TXB81);
		else SETBIT(UCSR1B,TXB81);

		sendPacket(&UDR1,&comm_state);

		if (comm_state.send_state==PS_READY) CLEARBIT(UCSR1B,UDRIE1);


}



// pøerušení - 50 Hz
ISR(TIMER0_COMP_vect) {


	// poèítadlo pro obnovení lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	C_SETBIT(F_50HZ);

	// aktualizace vnitrniho casu
	updateTime(&mod_state);

	// kontrola tlaèítek
	checkButtons(&mod_state);

	// poèítadlo timeoutu pro pøíjem po RS485
	receiveTimeout(&comm_state);

	// poèítadlo timeoutu pro pøíjem po RS232
	receiveTimeout(&pccomm_state);

	// 1 Hz
	if (++lcdc==50) {

		lcdc=0;

		// zvýšení poèítadla pro timeout komunikace s PC
		if (mod_state.pc_comm_to<PcCommTo) mod_state.pc_comm_to++;

		// nastavení pøíznakù
		C_SETBIT(F_1HZ);



	}



}




int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	mod_state.menu_state = M_STANDBY;

	// nastavení zdroje øízení na PC
	mod_state.control = C_PC;

	//pccomm_state.receive_state = PR_WAITING;

	// zjištìní PID konstant nastavených v EEPROM
	getMotorPID(10);


	// nekoneèná smyèka
    while (1) {

    	// reset watchdogu
    	wdt_reset();


    	// obsluha LCD
    	if (C_CHECKBIT(F_1HZ)) {

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

        	if (CHECKBIT(mod_state.buttons,ABUTT3)) {

        		if (++mod_state.control>C_PC) mod_state.control = C_AUTO;

        	    CLEARBIT(mod_state.buttons,ABUTT3);

        	}

        	if (CHECKBIT(mod_state.buttons,ABUTT4)) {

        		//  nekoneèná smyèka -> reset modulu
        		while(1);

        	}

    		C_CLEARBIT(F_1HZ);
    		manageLcd();


    	}


    	// obsluha periferií a podøízených modulù ---------------------------------------------------------------------------

    	if (C_CHECKBIT(F_50HZ)) {

    		C_CLEARBIT(F_50HZ);

			update_joystick();

			// ètení stavu levých motorù
			getMotorInfo(10,&motors.m[FRONT_LEFT],&motors.m[REAR_LEFT]);

			// ètení stavu pravých motorù
			getMotorInfo(11,&motors.m[FRONT_RIGHT],&motors.m[REAR_RIGHT]);

			// naètení dat ze senzorù
			getFastSensorState();

			// regulátor pro ujetou vzdálenost
			distReg();

			// regulátor otoèení
			angleReg();



		switch(mod_state.control) {

		// zdroj øízení nastaven na PC
		case C_PC: {

			// zastavení pøi timeoutu komunikace s PC - 5s
			if (mod_state.pc_comm_to >= PcCommTo)
				// pož. rychlost není nula - zastavit
				setMotorsSpeed(0,0);


		} break;

		// zdroj øízení nastaven na joystick
		case C_JOY: {

			joyRide();


		} break;

		// autonomní operace
		case C_AUTO: {

			// TODO: náhodná projížïka

			/*if (dist_reg.state == R_READY) {

				if (sens.us_fast>200) setDistReg(&dist_reg,sens.us_fast-200);

			} else {

				if (sens.us_fast > (dist_reg.req_dist+200)) setDistReg(&dist_reg,sens.us_fast-200);

			}*/

			// test otoèení o zadaný úhel
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

    	// crc souhlasí -> úspìšné pøijetí paketu
    	if ((pccomm_state.receive_state==PR_PACKET_RECEIVED) && checkPacket(&pccomm_state)) {

    		// vynulování poèítadla timeoutu kom. s PC
    		mod_state.pc_comm_to = 0;

    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zpìt stejný paket
    		case P_ECHO: {

    			// vytvoøení ECHO paketu
    			makePacket(&pccomm_state.op,pccomm_state.ip.data,pccomm_state.ip.len,P_ECHO,0);

    			// obsluha odeslání paketu
    			sendPCPacketE(&pccomm_state);


    		} break;

    		// volná jízda - ovládání joystickem
    		case PC_FREE_RIDE: {

    			int16_t spl, spr;

    			if (mod_state.control == C_PC ) {

					// rychlost pro levé motory
					spl = pccomm_state.ip.data[0];
					spl |= pccomm_state.ip.data[1]<<8;

					// rychlost pro pravé motory
					spr = pccomm_state.ip.data[2];
					spr |= pccomm_state.ip.data[3]<<8;

					setMotorsSpeed(spl,spr);


    			}

    		} break;

    		// požadavek na nastavení PID konstant
    		case P_MOTOR_SETPID: {

    			motors.P = pccomm_state.ip.data[0];
    			motors.I = pccomm_state.ip.data[1];
    			motors.D = pccomm_state.ip.data[2];

    			setMotorPID(10);
    			setMotorPID(11);


    		} break;

    		// požadavek na pøeètení PID konstant
    		case P_MOTOR_GETPID: {

    			uint8_t arr[3];

    			arr[0] = motors.P;
    		    arr[1] = motors.I;
    		    arr[2] = motors.D;

    		    makePacket(&pccomm_state.op,arr,3,P_MOTOR_GETPID,0);

    		    // obsluha odeslání paketu
    		    sendPCPacketE(&pccomm_state);


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

				// obsluha odeslání paketu
				sendPCPacketE();


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

    			// obsluha odeslání paketu
    			sendPCPacketE();

    		} break;

    		// plné mìøení - pouze když se stojí
    		case P_SENS_FULL: {

    			// provede se pouze když robot stojí a zdroj øízení je nastavený na PC
    			if ((motors.m[FRONT_LEFT].act_speed == 0) && (motors.m[FRONT_RIGHT].act_speed == 0) && (mod_state.control == C_PC)) {

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

    				// obsluha odeslání paketu
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
