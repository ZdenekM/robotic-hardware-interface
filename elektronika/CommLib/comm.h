#ifndef COMM_H
#define COMM_H


#define F_CPU 16000000UL

// prvn� synchroniza�n� byte
#define SYNC1 170

// druh� synchroniza�n� byte
#define SYNC2 171

// maxim�ln� po�et byt� v paketu
#define BUFF_LEN 32

// timeout pro p��jem paketu - 2x 10ms = 20ms
#define MAXTIMEOUT 10


typedef struct {

	// pole dat
	uint8_t data[BUFF_LEN];

	// d�lka dat
	uint8_t len;

	// typ paketu
	uint8_t packet_type;

	// kontroln� sou�et
	uint16_t crc;

	// adresa p��jemce
	uint8_t addr;

} tpacket;


// definice typ� paket� - 5. bajt paketu
// P_ECHO - odesl�n� echo paketu
// P_MOTOR_COMM - p��kaz pro motor
// P_MOTOR_INFO - informace z motor�
// P_COMM_INFO - informace o stavu komunikace

// PC_MOVE_STRAIGHT - pohyb rovn�
// PC_MOVE_ROUND - oto�en� o zadan� �hel
// PC_MOVE_INFO - informace o pohybu


typedef enum {P_ECHO, P_MOTOR_COMM,P_MOTOR_INFO,P_COMM_INFO,PC_FREE_RIDE, PC_MOVE_STRAIGHT, PC_MOVE_ROUND, PC_MOVE_INFO, PC_MOVE_AINFO, P_SENS_FAST,P_SENS_FULL,P_SENS_COMP,P_MOTOR_SETPID,P_MOTOR_GETPID} tpacket_type;

// definice stav� odes�l�n� paketu
typedef enum {PS_SYNC1, PS_SYNC2, PS_ADDR, PS_LEN, PS_TYPE, PS_DATA, PS_CRC1, PS_CRC2, PS_READY} tsend_state;

// definice stav� p��jmu paketu
typedef enum {PR_SYNC1, PR_SYNC2, PR_ADDR, PR_LEN, PR_TYPE, PR_DATA, PR_CRC1, PR_CRC2, PR_PACKET_RECEIVED, PR_TIMEOUT, PR_READY, PR_WAITING} treceive_state;


// struktura pro uchov�n� stavu komunikace a statistiky p�enosu
typedef struct {

	// stav vys�l�n�
	volatile tsend_state send_state;

	// stav p��jmu
	volatile treceive_state receive_state;

	// timeout pro p��jem paketu (po��tadlo milisekund)
	uint8_t receive_timeout;

	// paket k odesl�n�
	tpacket op;

	// po�et �sp�n� poslan�ch paket�
	uint32_t packets_sended;

	// po�et p�ijat�ch paket�
	uint32_t packets_received;

	// po�et paket� s vadn�m CRC
	uint16_t packets_bad_received;

	// kolikr�t nastal time-out
	uint16_t packets_timeouted;

	// kolikr�t do�lo p�i p��jmu bytu k chyb� r�mce
	uint16_t frame_error;

	// chyba synchronizace
	uint16_t sync_error;

	// p�ijat� paket
	tpacket ip;

	// adresa modulu
	uint8_t self_addr;

} tcomm_state;


// inicializace struktury typu tcomm_state
void comm_state_init(tcomm_state *c);

// spo��t�n� CRC pro pole o zadan� d�lce
extern uint16_t makeCRC(uint8_t *input, uint8_t len, uint8_t type, uint8_t addr);


// pomocn� funkce pro v�po�et CRC
extern uint16_t crc16_update(uint16_t crc, uint8_t a);

// sestaven� paketu pro komunikaci
extern void makePacket(tpacket *p, uint8_t *data,uint8_t len, uint8_t packet_type, uint8_t addr);

// zah�jen� p�enosu - odesl�n� prvn�ho bytu
extern void sendFirstByte(uint8_t *tUDR, tcomm_state *c);

// odesl�n� paketu
extern void sendPacket(uint8_t *tUDR, tcomm_state *c);

// p��jem paketu
extern void receivePacket(uint8_t tUDR, tcomm_state *c);

// obsluha po��tadla pro timeout
extern void receiveTimeout(tcomm_state *c);

// zkontroluje CRC paketu
extern uint8_t checkPacket(tcomm_state *c);

// uvoln�n� paketu
extern void freePacket(tpacket *p);

extern void pokus(tcomm_state *c);

#endif
