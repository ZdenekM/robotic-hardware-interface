#ifndef COMM_H
#define COMM_H


#define F_CPU 16000000UL

// první synchronizaèní byte
#define SYNC1 170

// druhý synchronizaèní byte
#define SYNC2 171

// maximální poèet bytù v paketu
#define BUFF_LEN 32

// timeout pro pøíjem paketu - 2x 10ms = 20ms
#define MAXTIMEOUT 10


typedef struct {

	// pole dat
	volatile uint8_t data[BUFF_LEN];

	// délka dat
	volatile uint8_t len;

	// typ paketu
	volatile uint8_t packet_type;

	// kontrolní souèet
	volatile uint16_t crc;

	// adresa pøíjemce
	volatile uint8_t addr;

} tpacket;


// definice typù paketù - 5. bajt paketu
// P_ECHO - odeslání echo paketu
// P_MOTOR_COMM - pøíkaz pro motor
// P_MOTOR_INFO - informace z motorù
// P_COMM_INFO - informace o stavu komunikace

// PC_MOVE_STRAIGHT - pohyb rovnì
// PC_MOVE_ROUND - otoèení o zadaný úhel
// PC_MOVE_INFO - informace o pohybu


typedef enum {P_ECHO, P_MOTOR_COMM,P_MOTOR_INFO,P_COMM_INFO,PC_MOVE_STRAIGHT, PC_MOVE_ROUND, PC_MOVE_INFO} tpacket_type;

// definice stavù odesílání paketu
typedef enum {PS_SYNC1, PS_SYNC2, PS_ADDR, PS_LEN, PS_TYPE, PS_DATA, PS_CRC1, PS_CRC2, PS_READY} tsend_state;

// definice stavù pøíjmu paketu
typedef enum {PR_SYNC1, PR_SYNC2, PR_ADDR, PR_LEN, PR_TYPE, PR_DATA, PR_CRC1, PR_CRC2, PR_PACKET_RECEIVED, PR_TIMEOUT, PR_READY, PR_WAITING, PR_BAD_CRC} treceive_state;


// struktura pro uchování stavu komunikace a statistiky pøenosu
typedef struct {

	// stav vysílání
	volatile tsend_state send_state;

	// stav pøíjmu
	volatile treceive_state receive_state;

	// timeout pro pøíjem paketu (poèítadlo milisekund)
	volatile uint8_t receive_timeout;

	// paket k odeslání
	volatile tpacket op;

	// poèet úspìšnì poslaných paketù
	volatile uint32_t packets_sended;

	// poèet pøijatých paketù
	volatile uint32_t packets_received;

	// poèet paketù s vadným CRC
	volatile uint16_t packets_bad_received;

	// kolikrát nastal time-out
	volatile uint16_t packets_timeouted;

	// kolikrát došlo pøi pøíjmu bytu k chybì rámce
	volatile uint16_t frame_error;

	// pøijatý paket
	volatile tpacket ip;

	// adresa modulu
	volatile uint8_t self_addr;

	// øetìzec identifikující modul
	volatile uint8_t id_str[15];

} tcomm_state;


// inicializace struktury typu tcomm_state
void comm_state_init(volatile tcomm_state *c);

// spoèítání CRC pro pole o zadané délce
extern uint16_t makeCRC(volatile uint8_t *input, uint8_t len, uint8_t type, uint8_t addr);


// pomocná funkce pro výpoèet CRC
extern uint16_t crc16_update(uint16_t crc, uint8_t a);

// sestavení paketu pro komunikaci
extern void makePacket(volatile tpacket *p, uint8_t *data,uint8_t len, uint8_t packet_type, uint8_t addr);

// zahájení pøenosu - odeslání prvního bytu
extern void sendFirstByte(volatile uint8_t *tUDR, volatile tcomm_state *c);

// odeslání paketu
extern void sendPacket(volatile uint8_t *tUDR, volatile tcomm_state *c);

// pøíjem paketu
extern void receivePacket(uint8_t tUDR, volatile tcomm_state *c);

// obsluha poèítadla pro timeout
extern void receiveTimeout(volatile tcomm_state *c);

// zkontroluje CRC paketu
extern uint8_t checkPacket(volatile tcomm_state *c);

// uvolnìní paketu
extern void freePacket(volatile tpacket *p);

extern void pokus(volatile tcomm_state *c);

#endif
