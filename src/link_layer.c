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

        enum State_SU state = START;
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
        enum State_SU state = START;
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

// This variable changes between 0 and 1
int last_sent = 0;
int llwrite(int fd, const unsigned char *buf, int bufSize, LinkLayer connectionParameters){
    // Construct information packet
    int packetSize = bufSize + 6;
    unsigned char *packet = (unsigned char *) malloc(packetSize);
    packet[0] = FLAG;
    packet[1] = A_NORMAL;
    packet[2] = last_sent ? C_I1 : C_I0;
    packet[3] = packet[1] ^ packet[2];
    // memcpy(packet+4, buf, bufSize); //Does we need this here? because we are going to stuff it

    unsigned char BCC2 = 0x00;
    int i = 4;
    for (unsigned int j = 0 ; i < bufSize ; j++) {
        // XOR operation
        BCC2 ^= buf[i]; 
        // Stuffing
        if(buf[i] == FLAG || buf[i] == ESCAPE){
            packet = (unsigned char *) realloc(packet, ++packetSize);
            packet[i++] = ESCAPE;
            packet[i++] = buf[i] ^ 0x20;
        }else{
            packet[i++] = buf[i];
        }
    }
    packet[i++] = BCC2;
    packet[i++] = FLAG;

    // Send packet
    int attempts = 0;
    while(attempts < connectionParameters.nRetransmissions){

        // activate alarm
        enable_alarm();
        alarmRing = FALSE;
        alarm(connectionParameters.timeout);

        int received = FALSE;    
        unsigned char cField = 0;

        while(!alarmRing && !received){
            if(write(fd, packet, packetSize) < 0){
                perror("write");
                exit(-1);
            }
            // Wait for RR
            unsigned char byte = 0;
            enum State_SU state = START;
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
                        if(byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_DISC){
                            state = C_RCV;
                            cField = byte;
                        }else if(byte == FLAG){
                            state = FLAG_RCV;
                        }else{
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if(byte == A_NORMAL ^ cField){
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
                            received = TRUE;
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
    
        // TODO: Verificar isto
        if(received){
            if(cField == C_RR0 || cField == C_RR1){
                last_sent = !last_sent;
                return bufSize;
            }else if(cField == C_REJ0 || cField == C_REJ1){
                attempts = 0;
            }else if(cField == C_DISC){
                return -1;
            }
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    return 0;
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

    enum State_SU state = START;
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
