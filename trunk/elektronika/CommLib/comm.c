
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <inttypes.h>
#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "comm.h"

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


uint16_t crc16_update(uint16_t crc, uint8_t a)
    {
        int i;

        crc ^= a;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = (crc >> 1);
        }

        return crc;
    }


uint16_t makeCRC(uint8_t *input, uint8_t len, uint8_t type, uint8_t addr)
{
    uint8_t i;
    uint16_t check;

    check=0;

    check = crc16_update(check,addr);
    check = crc16_update(check,len);
    check = crc16_update(check,type);

    if (input!=NULL && len > 0)
    for (i=0; i<len; i++)
        check = crc16_update(check,input[i]);


    return check;

}


// volá se z main
void makePacket(tpacket *p, uint8_t *data, uint8_t len, uint8_t packet_type, uint8_t addr) {

		p->addr = addr;
		p->len = len;
		p->packet_type = packet_type;

		p->crc = makeCRC(data,len,packet_type,addr);

		// kopírování dat
		if (len < BUFF_LEN && len > 0 && data !=NULL)
			memcpy(p->data,data,len);



}


// zahájení přenosu - odeslání prvního bytu
void sendFirstByte(uint8_t *tUDR, tcomm_state *c) {

	c->send_state = PS_SYNC1;
	*tUDR = SYNC1;

}


// funkce volaná z přerušení TX_Complete
// PS_SYNC1, PS_SYNC2, PS_ADDR, PS_LEN, PS_TYPE, PS_DATA, PS_CRC1, PS_CRC2, PS_READY
void sendPacket(uint8_t *tUDR, tcomm_state *c) {

	//static uint8_t index = 0;

	switch (c->send_state) {

		case PS_SYNC1: {
			// odeslání druhého s. bytu
			c->send_state = PS_SYNC2;
			*tUDR = SYNC2;

		} break;

		case PS_SYNC2: {
			// odeslání adresy příjemce
			c->send_state = PS_ADDR;
			*tUDR = c->op.addr;

		} break;

		case PS_ADDR: {
			// délka data
			c->send_state = PS_LEN;
			*tUDR = c->op.len;

		} break;

		case PS_LEN: {
			// přeskočení odesílání dat, pokud je délka 0
			if (c->op.len>0) c->send_state = PS_TYPE;
			else c->send_state = PS_DATA;
			// typ paketu
			*tUDR = c->op.packet_type;

		} break;

		case PS_TYPE: {
			// odeslání prvního bytu obsahu
			c->send_state = PS_DATA;
			*tUDR = c->op.data[c->tx_idx++];



		} break;

		case PS_DATA: {
			// odesílání obsahu paketu
			if (c->tx_idx < c->op.len) *tUDR = c->op.data[c->tx_idx++];
			else {
				// spodní byte CRC
				c->send_state = PS_CRC1;
				c->tx_idx = 0;
				// vynulovani horniho bytu
				*tUDR = (uint8_t)(c->op.crc&0xFF);

			}


		} break;

		case PS_CRC1: {
			// horní byte CRC
			//c->send_state = PS_CRC2;
			*tUDR = (uint8_t)(c->op.crc>>8);

			// konec přenosu
			c->send_state = PS_READY;
			c->packets_sended++;

		} break;

		/*case PS_CRC2: {
			// konec přenosu paketu
			c->send_state = PS_READY;
			c->packets_sended++;

		} break;*/

		//case PS_READY: break;


	}

}

// volá se před povolením přerušení
// inicializace struktury typu tcomm_state
void comm_state_init(tcomm_state *c) {

	if (c!=NULL) {

		c->send_state = PS_READY;
		c->receive_state = PR_READY;
		c->packets_sended = 0;
		c->receive_timeout = 0;
		c->packets_bad_received = 0;
		c->packets_received = 0;
		c->packets_timeouted = 0;
		c->frame_error = 0;
		c->sync_error = 0;


	}

}


// PR_SYNC1, PR_SYNC2, PR_ADDR, PR_LEN, PR_TYPE, PR_DATA, PR_CRC1, PR_CRC2, PR_PACKET_RECEIVED, PR_READY
void receivePacket(uint8_t tUDR, tcomm_state *c) {

	//static uint8_t index = 0;

	c->receive_timeout = 0;

	switch (c->receive_state) {

	case PR_WAITING: {

		if (tUDR==SYNC1) c->receive_state = PR_SYNC2;
		else {
			c->receive_state = PR_READY;
			c->sync_error++;
			}

	} break;



	case PR_SYNC2:  {

		if (tUDR==SYNC2) c->receive_state = PR_ADDR;
		else {
			c->receive_state = PR_READY;
			c->sync_error++;
		}


	} break;

	case PR_ADDR: {

		c->receive_state = PR_LEN;
		c->ip.addr = tUDR;


	} break;

	case PR_LEN: {

		if (tUDR<BUFF_LEN) {

			c->receive_state = PR_TYPE;
			c->ip.len = tUDR;

		} else	c->receive_state = PR_READY;


	} break;

	case PR_TYPE: {

		// přeskočení příjmu dat, pokud je délka dat 0
		if (c->ip.len>0) c->receive_state = PR_DATA;
		else c->receive_state = PR_CRC1;
		c->ip.packet_type = tUDR;

	} break;

	case PR_DATA: {

		c->ip.data[c->rec_idx++] = tUDR;

		// příjem dat dokončen
		if (c->rec_idx==c->ip.len) {
			c->receive_state = PR_CRC1;
			c->rec_idx = 0;
		}


	} break;

	case PR_CRC1: {

		c->receive_state = PR_CRC2;
		c->ip.crc = tUDR;


	} break;

	case PR_CRC2: {

		uint16_t p = tUDR;
		c->ip.crc |= (p<<8);

		c->receive_state = PR_PACKET_RECEIVED;

	} break;

	case PR_PACKET_RECEIVED: {

		// nedělá nic - příchozí byty se neberou v úvahu, dokud není někde zpracovaný aktuální paket

	} break;

	case PR_READY: {


	} break;




	} // switch


}

// volá se z main
// zkontroluje CRC paketu
uint8_t checkPacket(tcomm_state *c) {

	uint16_t crc = makeCRC(c->ip.data,c->ip.len,c->ip.packet_type,c->ip.addr);

	if (crc == c->ip.crc) {

		c->packets_received++;
		return 1;

	} else {

		// chyba CRC :-(
		c->packets_bad_received++;
		return 0;

	}


}

void receiveTimeout(tcomm_state *c) {

	 //&& c->receive_state!=PR_WAITING
	if (c->receive_state!=PR_PACKET_RECEIVED && c->receive_state!=PR_READY) {

		// chyba při přenosu
		if (c->receive_timeout++ > MAXTIMEOUT) {

			c->receive_timeout = 0;
			c->packets_timeouted++;
			c->receive_state = PR_TIMEOUT;

		}

	}

}



