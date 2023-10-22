#define FLAG 0x7E
#define A_NORMAL 0X03
#define A_REVERSE 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_I0 0x00
#define C_I1 0x40
#define ESCAPE 0x7D

enum LLState {START, FLAG_RCV, A_RCV, C_RCV, RECEIVING_DATA, STUFFING, BCC_OK, STOP};
