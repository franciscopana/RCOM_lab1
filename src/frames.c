// Protocol frames initialization and handling

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

/*
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

// SET normal
unsigned char set_packet[5] = {
    FLAG, 
    A_NORMAL, 
    C_SET, 
    A_NORMAL ^ C_SET, 
    FLAG
};
unsigned char *get_set_packet(){
    return set_packet;
}

// UA normal
unsigned char ua_packet[5] = {
    FLAG, 
    A_NORMAL, 
    C_UA, 
    A_NORMAL ^ C_UA, 
    FLAG
};
unsigned char *get_ua_packet(){
    return ua_packet;
}

// UA reverse
unsigned char ua_packet_reverse[5] = {
    FLAG, 
    A_REVERSE, 
    C_UA, 
    A_REVERSE ^ C_UA, 
    FLAG
};
unsigned char *get_ua_packet_reverse(){
    return ua_packet_reverse;
}

// DISC normal
unsigned char disc_packet[5] = {
    FLAG, 
    A_NORMAL, 
    C_DISC, 
    A_NORMAL ^ C_DISC, 
    FLAG
};
unsigned char *get_disc_packet(){
    return disc_packet;
}

// DISC reverse
unsigned char disc_packet_reverse[5] = {
    FLAG, 
    A_REVERSE, 
    C_DISC, 
    A_REVERSE ^ C_DISC, 
    FLAG
};
unsigned char *get_disc_packet_reverse(){
    return disc_packet_reverse;
}
