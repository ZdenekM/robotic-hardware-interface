#ifndef COMM_H
#define COMM_H


#define F_CPU 16000000UL

// první synchronizační byte
#define SYNC1 170

// druhý synchronizační byte
#define SYNC2 171

// maximální počet bytů v paketu
#define BUFF_LEN 32

// timeout pro příjem paketu - 2x 10ms = 20ms
#define MAXTIMEOUT 5


typedef struct {

	// pole dat
	uint8_t data[BUFF_LEN];

	// délka dat
	uint8_t len;

	// typ paketu
	uint8_t packet_type;

	// kontrolní součet
	uint16_t crc;

	// adresa příjemce
	uint8_t addr;

} tpacket;


// definice typů paketů - 5. bajt paketu
// P_ECHO - odeslání echo paketu
// P_MOTOR_COMM - příkaz pro motor
// P_MOTOR_INFO - informace z motorů
// P_COMM_INFO - informace o stavu komunikace

// PC_MOVE_STRAIGHT - pohyb rovně
// PC_MOVE_ROUND - otočení o zadaný úhel
// PC_MOVE_INFO - informace o pohybu


typedef enum {P_ECHO, P_MOTOR_COMM,P_MOTOR_INFO,P_COMM_INFO,PC_FREE_RIDE, PC_MOVE_STRAIGHT, PC_MOVE_ROUND, PC_MOVE_INFO, PC_MOVE_AINFO, P_SENS_FAST,P_SENS_FULL,P_SENS_COMP,P_MOTOR_SETPID,P_MOTOR_GETPID} tpacket_type;

// definice stavů odesílání paketu
typedef enum {PS_SYNC1, PS_SYNC2, PS_ADDR, PS_LEN, PS_TYPE, PS_DATA, PS_CRC1, PS_CRC2, PS_READY} tsend_state;

// definice stavů příjmu paketu
typedef enum {PR_SYNC1, PR_SYNC2, PR_ADDR, PR_LEN, PR_TYPE, PR_DATA, PR_CRC1, PR_CRC2, PR_PACKET_RECEIVED, PR_TIMEOUT, PR_READY, PR_WAITING} treceive_state;


// struktura pro uchování stavu komunikace a statistiky přenosu
typedef struct {

	// stav vysílání
	volatile tsend_state send_state;

	// stav příjmu
	volatile treceive_state receive_state;

	// timeout pro příjem paketu (počítadlo milisekund)
	uint8_t receive_timeout;

	// paket k odeslání
	tpacket op;

	// počet úspěšně poslaných paketů
	uint32_t packets_sended;

	// počet přijatých paketů
	uint32_t packets_received;

	// počet paketů s vadným CRC
	uint16_t packets_bad_received;

	// kolikrát nastal time-out
	uint16_t packets_timeouted;

	// kolikrát došlo při příjmu bytu k chybě rámce
	uint16_t frame_error;

	// chyba synchronizace
	uint16_t sync_error;

	// index pro příjem dat
	uint8_t rec_idx;

	uint8_t tx_idx;

	// přijatý paket
	tpacket ip;

	// adresa modulu
	uint8_t self_addr;

} tcomm_state;


// inicializace struktury typu tcomm_state
void comm_state_init(tcomm_state *c);

// spočítání CRC pro pole o zadané délce
extern uint16_t makeCRC(uint8_t *input, uint8_t len, uint8_t type, uint8_t addr);


// pomocná funkce pro výpočet CRC
extern uint16_t crc16_update(uint16_t crc, uint8_t a);

// sestavení paketu pro komunikaci
extern void makePacket(tpacket *p, uint8_t *data,uint8_t len, uint8_t packet_type, uint8_t addr);

// zahájení přenosu - odeslání prvního bytu
extern void sendFirstByte(uint8_t *tUDR, tcomm_state *c);

// odeslání paketu
extern void sendPacket(uint8_t *tUDR, tcomm_state *c);

// příjem paketu
extern void receivePacket(uint8_t tUDR, tcomm_state *c);

// obsluha počítadla pro timeout
extern void receiveTimeout(tcomm_state *c);

// zkontroluje CRC paketu
extern uint8_t checkPacket(tcomm_state *c);

// uvolnění paketu
extern void freePacket(tpacket *p);

extern void pokus(tcomm_state *c);

#endif
