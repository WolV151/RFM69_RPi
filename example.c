#include "RFM69.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define FREQUENCY       RF69_433MHZ     // Frequency of the radio
#define MY_NODE         1               // ID of this node within the network
#define PEER_NODE       2               // Node to send signals to
#define NETWORK_ID      100             // Network ID, all peers should use the same
#define INT_PIN         18              // RPi pin to attach interrupt to, connect to G0/D0 on the radio 
#define RST_PIN         29              // RPi pin connected to RST on the radio
#define SPI_BUS         0               // SPI bus to use


int main(void)
{
    initialize(FREQUENCY, MY_NODE, NETWORK_ID, INT_PIN, RST_PIN, SPI_BUS);
    printf("Radio Temperature: %d\n", readTemperature(0));
    char* string = "Hello World!";
    char* buf = "A";
    while(1)
    {
        if (receiveDone()) // if data is received, print the data and send a message to the peer
        {
            printf("Received message from node %d with RSSI %d\n", SENDERID, RSSI);

            if (ACKrequested())
            {
                sendACK(buf, 1);
                puts("ACK sent");
            }

            for(uint8_t i = 0; i < DATALEN; i++)
            {
                printf("%x", DATA[i]);
            }

            printf("\nSending message to peer\n");

            if (sendWithRetry(PEER_NODE, (uint8_t*)string, 12, 3, 200))
                puts("ACK received!");
            else
                puts("No ACK received.");

        }
    }
    return 0;
}
