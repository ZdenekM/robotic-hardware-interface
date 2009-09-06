// MainMod - kód pro řídicí modul
// autor: Zdeněk Materna, zdenek.materna@gmail.com
// stránky projektu: http://code.google.com/p/robotic-hardware-interface


// TODO: plné skenování okolí
// TODO: náhodná projížďka
// TODO: zastavení při výpadku SensMod
// TODO: zprovoznit watchdog
// TODO: podmenu na lcd

#include "MainMod.h"

// *** GLOBALNI PROMENNE **************************************************
// stavové proměnné modulu
tmod_state mod_state;

// stav komunikace - rs485
tcomm_state comm_state;

// stav komunikace - rs232
tcomm_state pccomm_state;

// strukt. pro regulátor ujeté vzdálenosti
tdist_reg dist_reg;

// reg. pro otáčení
tangle_reg angle_reg;

// struktura pro stav motorů
tmotors motors;

// flagy pro obsluhu perif.
volatile uint8_t flags = 0;

// stav senzorů
tsens sens;

// časové flagy
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



// přerušení - 50 Hz
ISR(TIMER0_COMP_vect) {


	// počítadlo pro obnovení lcd
	static uint8_t lcdc=0;

	//static uint8_t lcdbl=0;

	C_SETBIT(F_50HZ);

	// aktualizace vnitrniho casu
	updateTime(&mod_state);

	// kontrola tlačítek
	checkButtons(&mod_state);

	// počítadlo timeoutu pro příjem po RS485
	receiveTimeout(&comm_state);

	// počítadlo timeoutu pro příjem po RS232
	receiveTimeout(&pccomm_state);

	// 1 Hz
	if (++lcdc==50) {

		lcdc=0;

		// zvýšení počítadla pro timeout komunikace s PC
		if (mod_state.pc_comm_to<PcCommTo) mod_state.pc_comm_to++;

		// nastavení příznaků
		C_SETBIT(F_1HZ);



	}



}




int main(void)
{

	mod_state.menu_state = M_INIT;

	// inicializuje MainMod
	ioinit();

	mod_state.menu_state = M_STANDBY;

	// nastavení zdroje řízení na PC
	mod_state.control = C_PC;

	//pccomm_state.receive_state = PR_WAITING;

	// zjištění PID konstant nastavených v EEPROM
	getMotorPID(10);


	// nekonečná smyčka
    while (1) {

    	// reset watchdogu
    	wdt_reset();


    	// obsluha LCD
    	if (C_CHECKBIT(F_1HZ)) {

        	// přepínání režimu lcd ---------------------------------------------------------------------------
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

        		//setAngleReg(90);

        		while(comm_state.send_state != PS_READY);

        		// příkaz pro plné skenování
        		makePacket(&comm_state.op,NULL,0,P_SENS_FULL,21);
        		sendPacketE();

        		CLEARBIT(mod_state.buttons,ABUTT4);


        	}

    		C_CLEARBIT(F_1HZ);
    		manageLcd();


    	}


    	// obsluha periferií a podřízených modulů ---------------------------------------------------------------------------

    	if (C_CHECKBIT(F_50HZ)) {

    		C_CLEARBIT(F_50HZ);

			update_joystick();

			// čtení stavu levých motorů
			getMotorInfo(10,&motors.m[FRONT_LEFT],&motors.m[REAR_LEFT]);

			// čtení stavu pravých motorů
			getMotorInfo(11,&motors.m[FRONT_RIGHT],&motors.m[REAR_RIGHT]);

			// načtení dat ze senzorů
			getSensorState();

			// regulátor pro ujetou vzdálenost
			distReg();

			// regulátor otočení
			angleReg();



		switch(mod_state.control) {

		// zdroj řízení nastaven na PC
		case C_PC: {

			// zastavení při timeoutu komunikace s PC - 5s
			if (mod_state.pc_comm_to >= PcCommTo)
				// pož. rychlost není nula - zastavit
				setMotorsSpeed(0,0);


		} break;

		// zdroj řízení nastaven na joystick
		case C_JOY: {

			joyRide();


		} break;

		// autonomní operace
		case C_AUTO: {

			// TODO: náhodná projížďka

			/*if (dist_reg.state == R_READY) {

				if (sens.us_fast>200) setDistReg(&dist_reg,sens.us_fast-200);

			} else {

				if (sens.us_fast > (dist_reg.req_dist+200)) setDistReg(&dist_reg,sens.us_fast-200);

			}*/

			// test otočení o zadaný úhel
			/*static uint8_t flag = 0;

			if (flag==0) {

				flag = 1;

				setAngleReg(90);



			}*/

		} break;




		} //switch


    	} // 50 Hz



    	// obsluha komunikace s PC ---------------------------------------------------------------------------


		//pccomm_state.receive_state = PR_WAITING;

		//while (pccomm_state.receive_state!=PR_PACKET_RECEIVED && pccomm_state.receive_state!=PR_TIMEOUT && pccomm_state.receive_state!=PR_READY);

    	// crc souhlasí -> úspěšné přijetí paketu
    	if ((pccomm_state.receive_state==PR_PACKET_RECEIVED) && checkPacket(&pccomm_state)) {

    		// vynulování počítadla timeoutu kom. s PC
    		mod_state.pc_comm_to = 0;

    		switch (pccomm_state.ip.packet_type) {

    		//echo - poslat zpět stejný paket
    		case P_ECHO: {

    			// vytvoření ECHO paketu
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

    		// požadavek na přečtení PID konstant
    		case P_MOTOR_GETPID: {

    			uint8_t arr[3];

    			arr[0] = motors.P;
    		    arr[1] = motors.I;
    		    arr[2] = motors.D;

    		    makePacket(&pccomm_state.op,arr,3,P_MOTOR_GETPID,0);

    		    // obsluha odeslání paketu
    		    sendPCPacketE(&pccomm_state);


    		} break;


    		// jízda rovně
    		case PC_MOVE_STRAIGHT: {

    			int16_t dist;

    			dist = pccomm_state.ip.data[0];
    			dist |= pccomm_state.ip.data[1]<<8;

    			if (mod_state.control == C_PC && dist_reg.state == R_READY) setDistReg(dist);


    		} break;

    		// otočení
    		case PC_MOVE_ROUND: {

    		    // TODO: naprogramovat
    			if (mod_state.control == C_PC ) {



    			}


    		} break;

    		// informace o stavu pohonů
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

    			// TODO: dodělat

    		} break;

    		// data ze senzorů - měřeno za pohybu
    		case P_SENS_FAST: {

    			uint8_t arr[13];

    			// ultrazvuk - rychlé měření
    			arr[0] = sens.us_fast;
    			arr[1] = sens.us_fast>>8;

    			// sharp 1 (levý přední)
    			arr[2] = sens.sharp[0];
    			arr[3] = sens.sharp[0]>>8;

    			// sharp 2 (pravý přední)
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

    			// azimut z kompasu
    			arr[11] = sens.comp;
    			arr[12] = sens.comp>>8;

    			// vytvoření paketu
    			makePacket(&pccomm_state.op,arr,13,P_SENS_FAST,0);

    			// obsluha odeslání paketu
    			sendPCPacketE();

    		} break;

    		// plné měření - pouze když se stojí
    		case P_SENS_FULL: {

    			// provede se pouze když robot stojí a zdroj řízení je nastavený na PC
    			/*if ((motors.m[FRONT_LEFT].act_speed == 0) && (motors.m[FRONT_RIGHT].act_speed == 0) && (mod_state.control == C_PC)) {

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


					// sharp 1 (levý přední)
    				arr[10] = sens.sharp[0];
    				arr[11] = sens.sharp[0]>>8;

    				// sharp 2 (pravý přední)
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
    				sendPCPacketE();*/

    			//}



    		} break;


    		} // switch

    		pccomm_state.receive_state = PR_WAITING;

    	} // if


    	if ((pccomm_state.receive_state == PR_TIMEOUT) || (pccomm_state.receive_state == PR_READY)) pccomm_state.receive_state = PR_WAITING;



    	//_delay_ms(100);

    } // while

    return 0;
}
