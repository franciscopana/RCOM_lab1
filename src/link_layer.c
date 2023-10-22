// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include "link_layer.h"
#include "macros.h"

//TODO: move out of here
#define FALSE 0
#define TRUE 1

int alarmRing = FALSE;
int alarmCount = 0;
struct termios oldtio;


// Alarm function handler
void alarmHandler(int signal){
    alarmRing = TRUE;
    alarmCount++;
}

int enable_alarm(){
    (void)signal(SIGALRM, alarmHandler);
    return 0;
}
int disable_alarm(){
    (void)signal(SIGALRM, SIG_IGN);
    return 0;
}

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Open serial port
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0){
        perror(connectionParameters.serialPort);
        exit(-1);
    }

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
    
    //  transmitter sends SET and waits for UA using the retransmission mechanism    
    //  while attempt < MAX_ATTEMPTS
    //      send SET
    //      initialize alarm
    //      while !timeout
    //          wait for UA
    //          if UA received
    //              process_UA()
    //              return fd
    //  return -1.

    //  receiver waits for SET and sends UA

    if(connectionParameters.role == LlTx){
        unsigned char set_packet[5] = {0};
        set_packet[0] = FLAG;
        set_packet[1] = A_NORMAL;
        set_packet[2] = C_SET;
        set_packet[3] = A_NORMAL ^ C_SET;
        set_packet[4] = FLAG;

        enum LLState state = START;
        int attempts = 0;
        while(attempts < connectionParameters.nRetransmissions && state != STOP){
            // send set
            if(write(fd, set_packet, 5) < 0){
                perror("write");
                exit(-1);
            }
            // activate alarm
            enable_alarm();
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
                        if(byte == A_NORMAL ^ C_UA){
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
        disable_alarm();
        if(state != STOP) return -1;
        return fd;

    }else if(connectionParameters.role == LlRx){

        // wait for fully process SET
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
                    if(byte == A_NORMAL ^ C_SET){
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
        unsigned char ua_packet[5] = {0};
        ua_packet[0] = FLAG;
        ua_packet[1] = A_NORMAL;
        ua_packet[2] = C_UA;
        ua_packet[3] = A_NORMAL ^ C_UA;
        ua_packet[4] = FLAG;

        if(write(fd, ua_packet, 5) < 0){
            perror("write");
            exit(-1);
        }

        return fd;
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *packet)
{
    // RECEIVE PACKET I: F A C BCC1(A xor C) DATA BCC2(DATA0 xor DATA1 xor...) F
    // DATA is a sequence of bytes

    unsigned char byte, i_n;
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
                        i_n = 0;
                    }else if(byte == C_I1){
                        state = C_RCV;
                        i_n = 1;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(i_n = 0 && byte == A_NORMAL ^ C_I0){
                        state = RECEIVING_DATA;
                    }else if(i_n = 1 && byte == A_NORMAL ^ C_I1){
                        state = RECEIVING_DATA;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case RECEIVING_DATA:
                    if(byte == ESCAPE){
                        state = STUFFING;
                    }if(byte == FLAG){
                        unsigned char bcc2 = packet[packet_position--];
                        packet[packet_position] = '\0';
                        
                        int j = 0;
                        unsigned char xor = packet[j];
                        while(++j < packet_position){
                            xor ^= packet[j];
                        }

                        unsigned char supervision_packet[5] = {0};
                        supervision_packet[0] = FLAG;
                        supervision_packet[1] = A_NORMAL;
                        supervision_packet[3] = A_NORMAL ^ supervision_packet[2];
                        supervision_packet[4] = FLAG;

                        if(xor == bcc2){
                            state = STOP;
                            // send RR (i_n+1)%2
                            supervision_packet[2] = i_n == 0 ? C_RR1 : C_RR0;
                            write(fd, supervision_packet, 5);
                            return packet_position;
                        }else{
                            // send REJ (i_n)
                            supervision_packet[2] = i_n == 0 ? C_REJ0 : C_REJ1;
                            write(fd, supervision_packet, 5);
                        }
                    }else{
                        packet[packet_position++] = byte;
                    }
                    break;
                case STUFFING:
                    // TODO: check if this is correct
                    state = RECEIVING_DATA;
                    if(byte == ESCAPE || byte == FLAG){
                        packet[packet_position++] = byte;
                    }else{
                        packet[packet_position++] = ESCAPE;
                        packet[packet_position++] = byte;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd, int showStatistics, LinkLayer connectionParameters)
{
    // transmitter sends DISC and waits for DISC
    // transmitter sends UA
    unsigned char disc_packet[5] = {0};
    disc_packet[0] = FLAG;
    disc_packet[1] = A_NORMAL;
    disc_packet[2] = C_DISC;
    disc_packet[3] = A_NORMAL ^ C_DISC;
    disc_packet[4] = FLAG;

    enum LLState state = START;
    int attempts = 0;
    while(attempts < connectionParameters.nRetransmissions && state != STOP){
        // send disc
        if(write(fd, disc_packet, 5) < 0){
            perror("write");
            exit(-1);
        }
        // activate alarm
        enable_alarm();
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
                    if(byte == C_DISC){
                        state = C_RCV;
                    }else if(byte == FLAG){
                        state = FLAG_RCV;
                    }else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(byte == A_NORMAL ^ C_DISC){
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
    disable_alarm();
    if(state != STOP) return -1;

    //send UA
    unsigned char ua_packet[5] = {0};
    ua_packet[0] = FLAG;
    ua_packet[1] = A_NORMAL;
    ua_packet[2] = C_UA;
    ua_packet[3] = A_NORMAL ^ C_UA;
    ua_packet[4] = FLAG;

    if(write(fd, ua_packet, 5) < 0){
        perror("write");
        exit(-1);
    }

    //TODO: Restore things
    // close serial port
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    return 1;
}
