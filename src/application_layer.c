// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PAYLOAD 1000

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    if(strcmp(role, "tx") == 0)
        linkLayer.role = LlTx;
    else if (strcmp(role, "rx") == 0)
        linkLayer.role = LlRx;
    else{
        printf("Invalid role: %s\n", role);
        exit(1);
    }
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    // Establish connection
    int fd = llopen(linkLayer);
    if(fd < 0){
        printf("Error opening connection\n");
        exit(1);
    }

    // I am sending
    if(linkLayer.role == LlTx){
        //open file
        FILE *file = fopen(filename, "r");

        //get file size
        fseek(file, 0, SEEK_END);
        long int fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        // build control packet start
        // get the number of bytes to represent the file size
        unsigned char L1 = sizeof(fileSize);
        unsigned char L2 = strlen(filename);
        unsigned int controlPacketSize = 5 + L1 + L2;

        // build control packet
        unsigned char *startControlPacket = malloc(controlPacketSize);
        startControlPacket[0] = 0x02; // control field start
        startControlPacket[1] = 0x00; // control field end
        startControlPacket[2] = L1;  // T1
        /* Divide filesize in Bytes and put them on control packet*/
        for(int i = 0; i < L1; i++){
            startControlPacket[3 + i] = (fileSize >> (8 * i)) & 0xFF;
        }
        startControlPacket[3 + L1] = 0x01;
        startControlPacket[4 + L1] = L2;
        // Put filename on control packet
        memcpy(startControlPacket + 5 + L1, filename, L2);

        //read file
        char *fileBuffer = malloc(fileSize);
        fread(fileBuffer, fileSize, 1, file);
        fclose(file);

        //llwrite control packet
        llwrite(fd, startControlPacket, controlPacketSize, linkLayer);

        //loop buffer and send data packets
        long int bytesSent = 0;
        //TODO: malloc can be here, so we don't have to malloc everytime
        while(bytesSent < fileSize){
            long int bytesToSend = fileSize - bytesSent;
            if(bytesToSend > MAX_PAYLOAD){
                bytesToSend = MAX_PAYLOAD;
            }

            unsigned char* packet = malloc(bytesToSend + 3);
            packet[0] = 0x01; // control field data
            packet[1] = (bytesToSend >> 8) & 0xFF;
            packet[2] = bytesToSend & 0xFF;
            memcpy(packet + 3, fileBuffer + bytesSent, bytesToSend);
            llwrite(fd, packet, bytesToSend + 3, linkLayer);
            bytesSent += bytesToSend;
            free(packet);
        }
        free(fileBuffer);

        // build control packet end
        unsigned char *endControlPacket = malloc(controlPacketSize);
        memcpy(endControlPacket, startControlPacket, controlPacketSize);
        endControlPacket[0] = 0x03; // control field end

        //llwrite control packet
        llwrite(fd, endControlPacket, controlPacketSize, linkLayer);
        free(startControlPacket);
        free(endControlPacket);
        
        llclose(fd, 0, linkLayer);
    }
    
    // I am receiving
    else{
        //create buffer
        unsigned char *startCommandPacket = malloc(MAX_PAYLOAD);

        //receives control packet
        int controlPacketSize = llread(fd, startCommandPacket);

        printf("controlPacketSize: %d\n", controlPacketSize);

        //check if it is a start control packet
        if(startCommandPacket[0] != 0x02){
            printf("Invalid control packet\n");
            exit(1);
        }

        //get file size
        unsigned char L1 = startCommandPacket[2];
        long int fileSize = 0;
        for(int i = 0; i < L1; i++){
            fileSize |= startCommandPacket[3 + i] << (8 * i);
        }
        
        //get filename
        unsigned char L2 = startCommandPacket[3 + L1 + 1];
        char *filename = malloc(L2);
        memcpy(filename, startCommandPacket + 5 + L1, L2);

        //create a buffer to store the file
        unsigned char *fileBuffer = malloc(fileSize);
        int fileBufferSize = 0;

        //receives data packets
        while(1){
            unsigned char *packet = malloc(MAX_PAYLOAD);
            int packetSize = llread(fd, packet);

            //check if it is an end control packet
            if(packet[0] == 0x03){
                break;
            }

            //check if it is a data packet
            if(packet[0] != 0x01){
                printf("Invalid packet\n");
                exit(1);
            }

            //check if it is the expected packet
            if(packet[1] != ((fileBufferSize >> 8) & 0xFF) || packet[2] != ((fileBufferSize & 0xFF))){
                printf("Invalid packet\n");
                exit(1);
            }

            //copy data to buffer
            memcpy(fileBuffer + fileBufferSize, packet + 3, packetSize - 3);
            fileBufferSize += packetSize - 3;
            free(packet);
        }

        //TODO:
        //Write buffer to a new file with the name filename

        free(startCommandPacket);
        free(fileBuffer);
        free(filename);
    }
}
