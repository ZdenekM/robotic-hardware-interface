// MainMod - k�d pro ��dic� modul
// autor: Zden�k Materna, zdenek.materna@gmail.com
// str�nky projektu: http://code.google.com/p/robotic-hardware-interface
// comm.c -> funkce zaji��uj�c� komunikaci

#include "comm.h"

extern tcomm_state comm_state;
extern tcomm_state pccomm_state;


// vol� se z main
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

	// povolen� p�eru�en� UDRIE
	SETBIT(UCSR1B,UDRIE1);

	// �ek�n� na odesl�n� paketu
	while(comm_state.send_state != PS_READY);

	// �ek�n� na odesl�n� posledn�ho bytu
	while (!(UCSR1A & (1<<TXC1)));

	// vynulov�n� TXC1 - nastaven�m na jedni�ku
	SETBIT(UCSR1A,TXC1);

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

	for(sent=1; sent<=10;sent++) {

		// vytvo�en� paketu
		makePacket(&comm_state.op,data,30,P_ECHO,addr);

		// odesl�n� paketu
		sendPacketE();

		C_CLEARBIT(RS485_SEND);
		// �ek�n� na odpov��
		comm_state.receive_state = PR_WAITING;
		while(comm_state.receive_state != PR_PACKET_RECEIVED && comm_state.receive_state!=PR_TIMEOUT);

		// crc souhlas� -> �sp�n� p�ijet� paketu
		if (comm_state.receive_state==PR_PACKET_RECEIVED && checkPacket(&comm_state)) rec++;

		comm_state.receive_state = PR_READY;

	}

	// 10 pokus� -> rec*10 = �p�nost v %
	return rec*10;

}

// inicializace modul� - echo
void initModules() {

	lcd_gotoxy(0,0);
	lcd_puts_P("Testing modules...");

	char buff[15];
	uint8_t state;

	// MotorControl - left
	state = sendEcho(10);

	lcd_gotoxy(0,1);
	snprintf_P(buff,15,PSTR("MCL (10) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// MotorControl - right
	state = sendEcho(11);

	lcd_gotoxy(0,2);
	snprintf_P(buff,15,PSTR("MCR (11) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// SensMod
	state = sendEcho(21);

	lcd_gotoxy(0,3);
	snprintf_P(buff,15,PSTR("SEM (21) %3u\%"),state);
	lcd_puts(buff);


	_delay_ms(2000);
	lcd_clrscr();



}


// zobraz� na lcd statistiky komunikace
void commStat(tcomm_state *p) {

	char abuff[11];

	// odeslan�ch paket�
	lcd_gotoxy(0,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SE:%7u"),p->packets_sended);}
	lcd_puts(abuff);

	// po�et p�ijat�ch paket�
	lcd_gotoxy(0,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("RE:%7u"),p->packets_received);}
	lcd_puts(abuff);

	// po�et chyb r�mce
	lcd_gotoxy(0,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("FE:%7u"),p->frame_error);}
	lcd_puts(abuff);

	// po�et paket� s vadn�m CRC
	lcd_gotoxy(10,1);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("BR:%7u"),p->packets_bad_received);}
	lcd_puts(abuff);

	// po�et timeout�
	lcd_gotoxy(10,2);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("TO:%7u"),p->packets_timeouted);}
	lcd_puts(abuff);

	// chyba synchronizace
	lcd_gotoxy(10,3);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {sprintf_P(abuff,PSTR("SY:%7u"),p->sync_error);}
	lcd_puts(abuff);

}


// vol� se z main
// odesl�n� statistiky komunikace s moduly do PC
void sendCommStat() {

	/*uint8_t data[15]; // 1 byte typ, 14 byt� data
	data[0] = COMMSTATE;

	uint8_t index = 1;
	uint8_t i=0;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_sended>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(comm_state.packets_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_bad_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.packets_timeouted>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(comm_state.frame_error>>(i*8)); }

	makePacket(&pccomm_state.op,data,15,P_COMM_INFO,0);

	sendFirstByte(&UDR0,&pccomm_state);

	// �ek�n� na odesl�n� paketu
	while(pccomm_state.send_state != PS_READY);
	*/

}

void sendPCPacketE() {

	// �ek�n� na dokon�en� odesl�n� paketu
	while(pccomm_state.send_state != PS_READY);

	// zah�jen� p�enosu
	sendFirstByte(&UDR0,&pccomm_state);

	SETBIT(UCSR0B,UDRIE0);

}


