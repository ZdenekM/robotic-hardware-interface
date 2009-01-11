#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <inttypes.h>
#include <string.h>
#include <avr/io.h>
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


uint16_t makeCRC(uint8_t *input, uint8_t len)
{
    uint8_t i;
    uint16_t check;

    check=0;
    for (i=0; i<len; i++)
    {
        check = crc16_update(input[i], check);

    }

    return check;

}



void makePacket(tpacket *p, uint8_t *data,uint8_t len, uint8_t packet_type, uint8_t addr) {

		// alokování paketu


		p->addr = addr;
		p->len = len;
		p->packet_type = packet_type;
		p->crc = makeCRC(&data,len);

		// kopírování dat
		if (len < BUFF_LEN) memcpy(&p->data,&data,len);



}

// zahájení pøenosu - odeslání prvního bytu
void sendFirstByte(volatile uint8_t *tUDR, volatile tcomm_state *c) {

	c->send_state = PS_SYNC1;

	*tUDR = SYNC1;

}

// TODO: nastavování devátého bitu
// funkce volaná z pøerušení TX_Complete
// PS_SYNC1, PS_SYNC2, PS_ADDR, PS_LEN, PS_TYPE, PS_DATA, PS_CRC1, PS_CRC2, PS_READY
void sendPacket(volatile uint8_t *tUDR, volatile tcomm_state *c) {

	static uint8_t index = 0;

	switch (c->send_state) {

		case PS_SYNC1: {
			// odeslání druhého s. bytu
			c->send_state = PS_SYNC2;
			*tUDR = SYNC2;

		} break;

		case PS_SYNC2: {
			// odeslání adresy pøíjemce
			c->send_state = PS_ADDR;
			*tUDR = c->op.addr;

		} break;

		case PS_ADDR: {
			// délka data
			c->send_state = PS_LEN;
			*tUDR = c->op.len;

		} break;

		case PS_LEN: {
			// typ paketu
			c->send_state = PS_TYPE;
			*tUDR = c->op.packet_type;

		} break;

		case PS_TYPE: {
			// odeslání prvního bytu obsahu
			c->send_state = PS_DATA;
			*tUDR = c->op.data[index++];



		} break;

		case PS_DATA: {
			// odesílání obsahu paketu
			if (index < c->op.len) *tUDR = c->op.data[index++];
			else {
				// spodní byte CRC
				c->send_state = PS_CRC1;
				index = 0;
				// vynulovani horniho bytu
				*tUDR = (uint8_t)(c->op.crc&0xFF);

			}


		} break;

		case PS_CRC1: {
			// horní byte CRC
			c->send_state = PS_CRC2;
			*tUDR = (uint8_t)(c->op.crc>>8);

		} break;

		case PS_CRC2: {
			// konec pøenosu paketu
			c->send_state = PS_READY;
			c->packets_sended++;

		} break;

		//case PS_READY: break;


	}

}

// inicializace struktury typu tcomm_state
void comm_state_init(volatile tcomm_state *c) {

	if (c!=NULL) {

		c->send_state = PS_READY;
		c->receive_state = PR_READY;
		c->packets_sended = 0;
		c->receive_timeout = 0;
		c->packets_bad_received = 0;
		c->packets_received = 0;
		c->packets_timeouted = 0;
		c->frame_error = 0;


	}

}


// TODO: dodìlat pøíjem paketù podle 9. bitu
// PR_SYNC1, PR_SYNC2, PR_ADDR, PR_LEN, PR_TYPE, PR_DATA, PR_CRC1, PR_CRC2, PR_PACKET_RECEIVED, PR_READY
void receivePacket(uint8_t tUDR, volatile tcomm_state *c) {

	static uint8_t index = 0;

	c->receive_timeout = 0;

	//if (c!=NULL) {

	switch (c->receive_state) {

	case PR_READY: {

		if (tUDR==SYNC1) c->receive_state = PR_SYNC2;
		else c->receive_state = PR_READY;

	} break;

	case PR_SYNC2:  {

		if (tUDR==SYNC2) c->receive_state = PR_ADDR;
		else c->receive_state = PR_READY;


	} break;

	case PR_ADDR: {

		c->receive_state = PR_LEN;
		// TODO: porovnání pøijaté adresy s vlastní
		c->ip.addr = tUDR;


	} break;

	case PR_LEN: {

		if (tUDR<BUFF_LEN) {

			c->receive_state = PR_TYPE;
			c->ip.len = tUDR;

		} else	c->receive_state = PR_READY;


	} break;

	case PR_TYPE: {

		c->receive_state = PR_DATA;
		c->ip.packet_type = tUDR;

	} break;

	case PR_DATA: {

		c->ip.data[index++] = tUDR;

		// pøíjem dat dokonèen
		if (index==c->ip.len) {
			c->receive_state = PR_CRC1;
			index = 0;
		}


	} break;

	case PR_CRC1: {

		c->receive_state = PR_CRC2;
		c->ip.crc = tUDR;


	} break;

	case PR_CRC2: {

		uint16_t p = tUDR;
		c->ip.crc |= (p<<8);


		uint16_t crc = makeCRC(&c->ip.data,c->ip.len);

		if (crc == c->ip.crc) {

			c->receive_state = PR_PACKET_RECEIVED;
			c->packets_received++;

		} else {

			// chyba CRC :-(
			c->packets_bad_received++;
			c->receive_state = PR_READY;

		}

	} break;

	case PR_PACKET_RECEIVED: {

		// nedìlá nic - byty se neberou v úvahu, dokud není nìkde zpracovaný aktuální paket

	} break;




	} // switch

	//}

}

void receiveTimeout(volatile tcomm_state *c) {

	if (c->receive_state!=PR_PACKET_RECEIVED) {

		// chyba pøi pøenosu
		if (c->receive_timeout++ > MAXTIMEOUT) {

			c->receive_timeout = 0;
			c->packets_timeouted++;
			c->receive_state = PR_TIMEOUT;

		}

	}

}



