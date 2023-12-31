// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>


#define MAX_PAYLOAD 1000

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename){
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

        FILE *file = fopen(filename, "r");
        if(file == NULL){
            printf("File %s does not exist\n", filename);
            exit(1);
        }

        //get file size
        fseek(file, 0, SEEK_END);
        long int fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);
        printf("File size: %ld\n", fileSize);

        //get the number of bytes to represent the file size
        unsigned char L1 = sizeof(fileSize);
        unsigned char L2 = strlen(filename);
        unsigned int controlPacketSize = 5 + L1 + L2;

        //creating start control packet
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
        //copy filename to start control packet
        memcpy(startControlPacket + 5 + L1, filename, L2);

        //read file to a buffer
        char *fileBuffer = malloc(fileSize);
        fread(fileBuffer, fileSize, 1, file);
        fclose(file);

        //send start control packet
        if(llwrite(fd, startControlPacket, controlPacketSize, linkLayer) < 0){
            printf("Error sending packet\n");
            exit(1);
        }

        //loop buffer and send data packets
        long int bytesSent = 0;
        //IMPROVEMENT: malloc can be heres so we don't have to malloc everytime
        while(bytesSent < fileSize){
            unsigned long  bytesToSend = fileSize - bytesSent;
            if(bytesToSend > MAX_PAYLOAD){
                bytesToSend = MAX_PAYLOAD;
            }

            unsigned char* packet = malloc(bytesToSend + 3);
            packet[0] = 0x01; // control field data
            packet[1] = (bytesToSend >> 8) & 0xFF;
            packet[2] = bytesToSend & 0xFF;
            memcpy(packet + 3, fileBuffer + bytesSent, bytesToSend);
            if(llwrite(fd, packet, bytesToSend + 3, linkLayer) < 0){
                printf("Error sending packet\n");
                exit(1);
            }
            bytesSent += bytesToSend;
            free(packet);
            printf("file total bytes sent: %ld\n", bytesSent);
        }
        free(fileBuffer);

        //building control packet end by cloning start control packet
        unsigned char *endControlPacket = malloc(controlPacketSize);
        memcpy(endControlPacket, startControlPacket, controlPacketSize);
        endControlPacket[0] = 0x03; // control field end

        //send end control packet
        if(llwrite(fd, endControlPacket, controlPacketSize, linkLayer) < 0){
            printf("Error sending packet\n");
            exit(1);
        }
        free(startControlPacket);
        free(endControlPacket);
        
        llclose(fd, 0, linkLayer);
    }
    
    // I am receiving
    else{
        clock_t start = clock();

        //create buffer
        unsigned char *startCommandPacket = malloc(MAX_PAYLOAD);

        //receives control packet
        llread(fd, startCommandPacket);

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
        printf("File size: %ld\n", fileSize);
        
        //get filename
        unsigned char L2 = startCommandPacket[3 + L1 + 1];
        char *filename = malloc(L2);
        memcpy(filename, startCommandPacket + 5 + L1, L2);
        printf("Filename: %s\n", filename);

        char *appendToFilename = "-received";
        size_t originalLength = strlen(filename);
        size_t appendLength = strlen(appendToFilename);
        size_t newFilenameLength = originalLength + appendLength;
        char *newFilename = (char *)malloc(newFilenameLength + 1);
        strcpy(newFilename, filename);
        char* dot = strrchr(newFilename, '.');
        if(dot != NULL){
            size_t position = dot - newFilename;
            memmove(newFilename + position + appendLength, dot, originalLength - position);
            memcpy(newFilename + position, appendToFilename, appendLength);
        }else{
            strcat(newFilename, appendToFilename);
        }
        printf("New filename: %s\n", newFilename);

        //if the file already exists, delete it
        if(access(newFilename, F_OK) != -1){
            remove(newFilename);
        }
        FILE *file = fopen(newFilename, "a");

        int bytesWritten = 0;
        //receives data packets
        while(1){
            unsigned char *packet = malloc(MAX_PAYLOAD + 3);
            int packetSize = llread(fd, packet);
            if(packetSize < 0){
                printf("Error receiving packet\n");
                exit(1);
            }else if(packetSize == 0){
                free(packet);
                continue;
            }

            packet = realloc(packet, packetSize);

            //check if it is an end control packet
            if(packet[0] == 0x03){
                printf("End control packet received\n");
                break;
            }

            //check if it is a data packet
            if(packet[0] != 0x01){
                printf("Invalid packet1\n");
                exit(1);
            }

            // packet: [C | L2 | L1 | P1 | ... | Pk]
            // C = 0x01, L2L1 = k

            unsigned char L2 = packet[1];
            unsigned char L1 = packet[2];
            unsigned int k = (L2 << 8) | L1;

            printf("Data Package received\n");
            fwrite(packet + 3, k, 1, file);
            printf("BytesWritten1: %d\n", bytesWritten);
            bytesWritten += k;
            free(packet);
        }
        clock_t end = clock();
        double timeSpent = (double)(end - start) / CLOCKS_PER_SEC;
        printf("Time spent: %f\n", timeSpent);

        llclose(fd, 0, linkLayer);

        //free memory
        free(newFilename);
        free(startCommandPacket);
        fclose(file);

    }
}
