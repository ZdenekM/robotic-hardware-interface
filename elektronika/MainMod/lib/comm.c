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
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <avr/wdt.h>

#include "comm.h"


// vol� se z main
// odesl�n� paketu - extra funkce pro ka�d� modul
void sendPacketE(tcomm_state *c) {

	// zak�z�n� p��jmu
	CLEARBIT(UCSR1B,RXEN1);

	// p�epnut� na vys�l�n�
	C_SETBIT(RS485_SEND);

	// nastaven� 9. bitu
	SETBIT(UCSR1B,TXB81);

	// posl�n� prvn�ho bytu - ostatn� se vys�laj� automaticky
	sendFirstByte(&UDR1,&c);

	// povolen� p�eru�en� UDRIE
	SETBIT(UCSR1B,UDRIE1);

	// �ek�n� na odesl�n� paketu
	while(c->send_state != PS_READY);

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
uint8_t sendEcho(tcomm_state *c, uint8_t addr) {

	uint8_t data[30], i=0,sent=0,rec=0;

	for(i=0; i<30;i++) data[i] = i*2;

	for(sent=1; sent<=10;sent++) {

		// vytvo�en� paketu
		makePacket(&c->op,data,30,P_ECHO,addr);

		// odesl�n� paketu
		sendPacketE(&c);

		C_CLEARBIT(RS485_SEND);
		// �ek�n� na odpov��
		c->receive_state = PR_WAITING;
		while(c->receive_state != PR_PACKET_RECEIVED && c->receive_state!=PR_TIMEOUT);

		// crc souhlas� -> �sp�n� p�ijet� paketu
		if (c->receive_state==PR_PACKET_RECEIVED && checkPacket(&c)) rec++;

		c->receive_state = PR_READY;

	}

	// 10 pokus� -> rec*10 = �p�nost v %
	return rec*10;

}

// inicializace modul� - echo
void initModules(tcomm_state *c) {

	lcd_gotoxy(0,0);
	lcd_puts_P("Testing modules...");

	char buff[15];
	uint8_t state;

	// MotorControl - left
	state = sendEcho(&c,10);

	lcd_gotoxy(0,1);
	snprintf_P(buff,15,PSTR("MCL (10) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// MotorControl - right
	state = sendEcho(&c,11);

	lcd_gotoxy(0,2);
	snprintf_P(buff,15,PSTR("MCR (11) %3u\%"),state);
	lcd_puts(buff);

	_delay_ms(1);

	// SensMod
	state = sendEcho(&c,21);

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
void sendCommStat(tcomm_state *c, tcomm_state *pc) {

	uint8_t data[15]; // 1 byte typ, 14 byt� data
	data[0] = COMMSTATE;

	uint8_t index = 1;
	uint8_t i=0;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(c->packets_sended>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<4;i++)
		data[index++]=(uint8_t)(c->packets_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(c->packets_bad_received>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(c->packets_timeouted>>(i*8)); }

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	for (i=0;i<2;i++)
		data[index++]=(uint8_t)(c->frame_error>>(i*8)); }

	makePacket(&pc->op,data,15,P_COMM_INFO,0);

	sendFirstByte(&UDR0,&pc);

	// �ek�n� na odesl�n� paketu
	while(pc->send_state != PS_READY);


}

void sendPCPacketE(tcomm_state *pc) {

	// �ek�n� na dokon�en� odesl�n� paketu
	while(pc->send_state != PS_READY);

	// zah�jen� p�enosu
	sendFirstByte(&UDR0,&pc);

	SETBIT(UCSR0B,UDRIE0);

}


