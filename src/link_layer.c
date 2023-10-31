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


// TODO-REFACTOR
/* Generating packets file
- UA normal
- UA reverse
- DISC normal
- DISC reverse
- SET normal
- RR0
- RR1
- REJ0
- REJ1


*/

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

    // TODO-REFACTOR: Move this to an independent function
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

    // TODO-REFACTOR make a switch
    if(connectionParameters.role == LlTx){
        unsigned char set_packet[5] = {0};
        set_packet[0] = FLAG;
        set_packet[1] = A_NORMAL;
        set_packet[2] = C_SET;
        set_packet[3] = A_NORMAL ^ C_SET;
        set_packet[4] = FLAG;

        //TODO-REFACTOR: Create a generic state machine function
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
        disable_alarm();
        if(state != STOP) return -1;
        printf("Connection established\n");
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

int llwrite(int fd, const unsigned char *buf, int bufSize, LinkLayer connectionParameters){

    static int sequenceNumber = 0;

    // Construct information packet
    int packetSize = bufSize + 6;
    unsigned char *packet = (unsigned char *) malloc(packetSize);
    packet[0] = FLAG;
    packet[1] = A_NORMAL;
    packet[2] = sequenceNumber == 0 ? C_I0 : C_I1;
    packet[3] = packet[1] ^ packet[2];

    unsigned char BCC2 = 0x00;
    int i = 4;
    // print buffer size
    printf("bufSize: %d\n", bufSize);
    for (unsigned int j = 0 ; j < bufSize ; j++) {
        // XOR operation
        BCC2 ^= buf[j]; 

        //TODO-REFACTOR: Create a stuffing and destuffing function
        // Stuffing
        if(buf[j] == FLAG || buf[j] == ESCAPE){
            printf("Stuffing\n");
            packet = (unsigned char *) realloc(packet, ++packetSize);
            packet[i++] = ESCAPE;
            packet[i++] = buf[j] ^ 0x20;
        }else{
            packet[i++] = buf[j];
        }
    }

    // BCC2 Stuffing
    if(BCC2 == FLAG || BCC2 == ESCAPE){
        printf("Stuffing\n");
        packet = (unsigned char *) realloc(packet, ++packetSize);
        packet[i++] = ESCAPE;
        packet[i++] = BCC2 ^ 0x20;
    }else{
        packet[i++] = BCC2;
    }
    packet[i++] = FLAG;

    printf("packetSize: %d\n", packetSize);

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
                        if(byte == (A_NORMAL ^ cField)){
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
    
        if(received){
            if(cField == C_RR0 || cField == C_RR1){
                sequenceNumber = (sequenceNumber+1) % 2;
                return bufSize;
            }else if(cField == C_REJ0 || cField == C_REJ1){
                //atempts = 0;
                attempts--;
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
int llread(int fd, unsigned char *packet){
    // RECEIVE PACKET I: F A C BCC1(A xor C) DATA BCC2(DATA0 xor DATA1 xor...) F
    // DATA is a sequence of bytes

    static char sequenceNumber = 0;

    unsigned char byte, sequenceNumberReceived;
    int packet_position = 0;
    enum LLState state = START;

    while (state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch(state){
                case START:
                    printf("start state\n");
                    if(byte == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    printf("flag_rcv state\n");
                    if(byte == A_NORMAL){
                        state = A_RCV;
                    }else if(byte != FLAG){
                        state = START;
                    }
                    break;
                case A_RCV:
                    printf("a_rcv state\n");
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
                    printf("c_rcv state\n");
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
                    //printf("RECEIVING_DATA\n");
                    if(byte == ESCAPE){
                        state = DESTUFFING;
                    }else if(byte == FLAG){
                        unsigned char bcc2 = packet[--packet_position];
                        packet[packet_position] = '\0';
                        
                        int j = 0;
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
                        }
                    }else{
                        packet[packet_position++] = byte;
                    }
                    break;
                case DESTUFFING:
                    printf("DESTUFFING\n");
                    /*
                    STUFFING:qwdx
                    byte FLAG(0x7e) replaced for ESCAPE(0x7d) followed by 0x5e
                    byte ESCAPE(0x7d) replaced for ESCAPE(0x7d) followed by 0x5d

                    Here we want to do the reverse process

                                 p_pos   byte        
                    | 0X7D | -> | 0x7d | 0x5d |
                    | 0X7E | -> | 0x7d | 0x5e |
                    */

                    if(byte == 0x5e){
                        packet[packet_position++] = FLAG;
                    }else if(byte == 0x5d){
                        packet[packet_position++] = ESCAPE;
                    }else{
                        packet[packet_position++] = byte;
                    }
                    state = RECEIVING_DATA;
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

    if(connectionParameters.role == LlTx){
        printf("Entered llclose on transmitter\n");
        // transmitter sends DISC and waits for DISC then sends UA

        unsigned char disc_packet[5] = {0};
        disc_packet[0] = FLAG;
        disc_packet[1] = A_NORMAL;
        disc_packet[2] = C_DISC;
        disc_packet[3] = A_NORMAL ^ C_DISC;
        disc_packet[4] = FLAG;

        enum LLState state = START;
        int attempts = 0;
        while(attempts < connectionParameters.nRetransmissions && state != STOP){
            //print the attempt
            printf("attempt: %d\n", attempts);
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
        disable_alarm();
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
        unsigned char ua_packet[5] = {0};
        ua_packet[0] = FLAG;
        ua_packet[1] = A_REVERSE;
        ua_packet[2] = C_UA;
        ua_packet[3] = A_REVERSE ^ C_UA;
        ua_packet[4] = FLAG;

        if(write(fd, ua_packet, 5) < 0){
            perror("write");
            exit(-1);
        }
        printf("Transmitter sent UA\n");
    }
    else{
        // receiver waits for DISC and sends DISC then waits for UA
        printf("Entered llclose on receiver\n");
        // wait for fully process DISC
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
        unsigned char disc_packet[5] = {0};
        disc_packet[0] = FLAG;
        disc_packet[1] = A_REVERSE;
        disc_packet[2] = C_DISC;
        disc_packet[3] = A_REVERSE ^ C_DISC;
        disc_packet[4] = FLAG;

        state = START;
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
        disable_alarm();
        if(state != STOP) return -1;

        printf("Receiver received UA from Transmitter\n");
    }

    // Restore things back
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    printf("Connection closed\n");
    return 1;
}
