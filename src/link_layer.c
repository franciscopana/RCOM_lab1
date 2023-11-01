// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <unistd.h>
#include <signal.h>

#include "link_layer.h"
#include "macros.h"
#include "frames.h"

int alarmRing = FALSE;
struct termios oldtio;

//Private functions declarations
void setup_new_termios(int fd, LinkLayer connectionParameters);
void reset_old_termios(int fd);
void alarmHandler(int signal);
int enable_alarm();
int disable_alarm();
void stuffing(unsigned char *frame, int *frameSize, int *i, unsigned char byte);
void destuffing(unsigned char *frame, int *frameSize, unsigned char byte);

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// TODO-REFACTOR
/* Generating packets file
- RR0
- RR1
- REJ0
- REJ1
*/

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    enable_alarm();

    // Open serial port
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0){
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    setup_new_termios(fd, connectionParameters);

    if(connectionParameters.role == LlTx){

        //TODO-REFACTOR: Create a generic state machine function
        enum LLState state = START;
        int attempts = 0;
        while(attempts < connectionParameters.nRetransmissions && state != STOP){
            // send set
            if(write(fd, get_set_packet(), 5) < 0){
                perror("write");
                exit(-1);
            }
            // activate alarm
            alarmRing = FALSE;
            alarm(connectionParameters.timeout);
            
            while(!alarmRing && state != STOP){
                unsigned char byte;
                if(read(fd, &byte, 1) < 0){
                    perror("read");
                    exit(-1);
                }

                switch(state){
                    case START:
                        if(byte == FLAG){
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if(byte == A_NORMAL){
                            state = A_RCV;
                        }else if(byte != FLAG){
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if(byte == C_UA){
                            state = C_RCV;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if(byte == (A_NORMAL ^ C_UA)){
                            state = BCC_OK;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if(byte == FLAG){
                            state = STOP;
                        }else{
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            } 
            attempts++;
        }
        if(state != STOP) return -1;
        printf("Connection established\n");
        return fd;

    }else if(connectionParameters.role == LlRx){

        //Wait for fully process SET and send UA
        unsigned char byte;
        enum LLState state = START;
        while(state != STOP){
            if(read(fd, &byte, 1) < 0){
                perror("read");
                exit(-1);
            }

            switch(state){
                case START:
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(byte == A_NORMAL){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(byte == C_SET){
                        state = C_RCV;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(byte == (A_NORMAL ^ C_SET)){
                        state = BCC_OK;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if(byte == FLAG){
                        state = STOP;
                    }else{
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }

        // send UA
        unsigned char *ua_packet = get_ua_packet();
        if(write(fd, ua_packet, 5) < 0){
            perror("write");
            exit(-1);
        }

        return fd;
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(int fd, const unsigned char *buf, int bufSize, LinkLayer connectionParameters){
    static int sequenceNumber = 0;
    sleep(1);

    int frameSize = bufSize + 6;
    unsigned char *frame = (unsigned char *) malloc(frameSize);
    frame[0] = FLAG;
    frame[1] = A_NORMAL;
    frame[2] = sequenceNumber == 0 ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];

    unsigned char BCC2 = 0x00;
    int i = 4;
    for (unsigned int j = 0 ; j < bufSize ; j++) {
        BCC2 ^= buf[j]; // XOR operation
        stuffing(frame, &frameSize, &i, buf[j]);
    }
    // BCC2 Stuffing
    stuffing(frame, &frameSize, &i, BCC2);
    frame[i++] = FLAG;

    int attempts = 0;
    while(attempts < connectionParameters.nRetransmissions){
        //Send packet
        if(write(fd, frame, frameSize) < 0){
            perror("write");
            exit(-1);
        }

        alarmRing = FALSE;
        alarm(connectionParameters.timeout);

        //wait for response
        unsigned char ack;
        enum LLState state = START;
        while(!alarmRing && state != STOP){
            unsigned char byte;
            if(read(fd, &byte, 1) < 0){
                perror("read");
                exit(-1);
            }

            switch(state){
                case START:
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(byte == A_NORMAL){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1){
                        ack = byte;
                        state = C_RCV;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(byte == (A_NORMAL ^ ack)){
                        state = BCC_OK;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if(byte == FLAG){
                        state = STOP;
                    }else{
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }
        if(state == STOP && (ack == C_RR0 || ack == C_RR1)){
            sequenceNumber = (sequenceNumber + 1) % 2;
            return bufSize;
        }
        attempts++;
    }
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *packet){
    // RECEIVE PACKET I: 
    // [F | A | C | BCC1(A xor C) | DATA1 | ... | DATAk | BCC2(DATA0 xor DATA1 xor...) | F]

    static char sequenceNumber = 0;

    unsigned char byte, sequenceNumberReceived;
    int packet_position = 0;
    enum LLState state = START;

    while (state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch(state){
                case START:
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(byte == A_NORMAL){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(byte == C_I0){
                        state = C_RCV;
                        sequenceNumberReceived = 0;
                    }else if(byte == C_I1){
                        state = C_RCV;
                        sequenceNumberReceived = 1;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(sequenceNumberReceived == 0 && byte == (A_NORMAL ^ C_I0)){
                        state = RECEIVING_DATA;
                    }else if(sequenceNumberReceived == 1 && byte == (A_NORMAL ^ C_I1)){
                        state = RECEIVING_DATA;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case RECEIVING_DATA:
                    if(byte == ESCAPE){
                        state = DESTUFFING;
                    }else if(byte == FLAG){
                        unsigned char bcc2 = packet[--packet_position];
                        packet[packet_position] = '\0';
                        
                        unsigned char xor = 0x00;
                        for(int j = 0; j < packet_position; j++){
                            xor ^= packet[j];
                        }

                        unsigned char supervision_packet[5] = {0};
                        supervision_packet[0] = FLAG;
                        supervision_packet[1] = A_NORMAL;
                        supervision_packet[4] = FLAG;

                        if(xor == bcc2){
                            state = STOP;
                            // send RR
                            supervision_packet[2] = sequenceNumberReceived == 0 ? C_RR1 : C_RR0;
                            supervision_packet[3] = A_NORMAL ^ supervision_packet[2];
                            write(fd, supervision_packet, 5);

                            // check if it is not a repeated packet
                            if(sequenceNumberReceived == sequenceNumber){
                                sequenceNumber = (sequenceNumber + 1) % 2;
                                return packet_position;
                            }
                            return 0;
                        }else{
                            // send REJ (i_n)
                            supervision_packet[2] = sequenceNumberReceived == 0 ? C_REJ0 : C_REJ1;
                            supervision_packet[3] = A_NORMAL ^ supervision_packet[2];
                            write(fd, supervision_packet, 5);
                            state = START;
                        }
                    }else{
                        packet[packet_position++] = byte;
                    }
                    break;
                case DESTUFFING:
                    destuffing(packet, &packet_position, byte);
                    state = RECEIVING_DATA;
                    break;
                default:
                    state = START;
                    break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd, int showStatistics, LinkLayer connectionParameters){

    if(connectionParameters.role == LlTx){
        printf("Entered llclose on transmitter\n");
        
        //transmitter sends DISC and waits for DISC then sends UA
        unsigned char *disc_packet = get_disc_packet();

        enum LLState state = START;
        int attempts = 0;
        while(attempts < connectionParameters.nRetransmissions && state != STOP){
            printf("Attempt: %d\n", attempts);

            //send disc packet
            if(write(fd, disc_packet, 5) < 0){
                perror("write");
                exit(-1);
            }
            //activate alarm
            alarmRing = FALSE;
            alarm(connectionParameters.timeout);
            
            while(!alarmRing && state != STOP){
                unsigned char byte;
                if(read(fd, &byte, 1) < 0){
                    perror("read");
                    exit(-1);
                }

                switch(state){
                    case START:
                        if(byte == FLAG){
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if(byte == A_REVERSE){
                            state = A_RCV;
                        }else if(byte != FLAG){
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if(byte == C_DISC){
                            state = C_RCV;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if(byte == (A_REVERSE ^ C_DISC)){
                            state = BCC_OK;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if(byte == FLAG){
                            state = STOP;
                        }else{
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            } 
            attempts++;
        }
        if(state != STOP) return -1;

        // wait for fully process DISC from receiver
        state = START;
        char byte;
        while(state != STOP){
            if(read(fd, &byte, 1) < 0){
                perror("read");
                exit(-1);
            }

            switch(state){
                case START:
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(byte == A_REVERSE){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(byte == C_DISC){
                        state = C_RCV;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(byte == (A_REVERSE ^ C_DISC)){
                        state = BCC_OK;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if(byte == FLAG){
                        state = STOP;
                    }else{
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }

        printf("Transmitter received DISC from Receiver\n");

        //send UA
        unsigned char *ua_packet = get_ua_packet_reverse();
        if(write(fd, ua_packet, 5) < 0){
            perror("write");
            exit(-1);
        }
        printf("Transmitter sent UA\n");
    }
    else{
        printf("Entered llclose on receiver\n");
        //receiver waits for DISC and sends DISC then waits for UA
        unsigned char byte;
        enum LLState state = START;
        while(state != STOP){
            //printf("waiting\n");
            if(read(fd, &byte, 1) < 0){
                perror("read");
                exit(-1);
            }

            switch(state){
                case START:
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(byte == A_NORMAL){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(byte == C_DISC){
                        state = C_RCV;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(byte == (A_NORMAL ^ C_DISC)){
                        state = BCC_OK;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if(byte == FLAG){
                        state = STOP;
                    }else{
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }

        printf("Receiver received DISC from Transmitter\n");
        // send DISC with timeout and retransmission mechanism waiting for UA
        unsigned char *disc_packet = get_disc_packet_reverse();

        state = START;
        int attempts = 0;
        while(attempts < connectionParameters.nRetransmissions && state != STOP){
            //Send disc
            if(write(fd, disc_packet, 5) < 0){
                perror("write");
                exit(-1);
            }

            alarmRing = FALSE;
            alarm(connectionParameters.timeout);
            
            while(!alarmRing && state != STOP){
                unsigned char byte;
                if(read(fd, &byte, 1) < 0){
                    perror("read");
                    exit(-1);
                }

                switch(state){
                    case START:
                        if(byte == FLAG){
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if(byte == A_REVERSE){
                            state = A_RCV;
                        }else if(byte != FLAG){
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if(byte == C_UA){
                            state = C_RCV;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if(byte == (A_REVERSE ^ C_UA)){
                            state = BCC_OK;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if(byte == FLAG){
                            state = STOP;
                        }else{
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            }
            attempts++;
        }
        if(state != STOP) return -1;

        printf("Receiver received UA from Transmitter\n");
    }

    printf("Connection closed\n");
    reset_old_termios(fd);
    disable_alarm();
    return 1;
}

void alarmHandler(int signal){
    printf("Alarm ringing!\n");
    alarmRing = TRUE;
}
int enable_alarm(){
    (void)signal(SIGALRM, alarmHandler);
    return 0;
}
int disable_alarm(){
    (void)signal(SIGALRM, SIG_IGN);
    return 0;
}

void setup_new_termios(int fd, LinkLayer connectionParameters){
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1){
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }
    printf("New termios structure set\n");
}

void reset_old_termios(int fd){
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
}

void stuffing(unsigned char *frame, int *frameSize, int *i, unsigned char byte){
    if(byte == FLAG || byte == ESCAPE){
        frame = (unsigned char *) realloc(frame, ++(*frameSize));
        frame[(*i)++] = ESCAPE;
        frame[(*i)++] = byte ^ 0x20;
    }else{
        frame[(*i)++] = byte;
    }
}

void destuffing(unsigned char *frame, int *frameSize, unsigned char byte){
    if(byte == 0x5e){
        frame[(*frameSize)++] = FLAG;
    }else if(byte == 0x5d){
        frame[(*frameSize)++] = ESCAPE;
    }else{
        frame[(*frameSize)++] = byte;
    }
}