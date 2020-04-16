#include "RFM69.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FREQUENCY     RF69_868MHZ

int main(void)
{
    initialize(FREQUENCY, 1, 100, 16, 22, 0);
    printf("Temperatur: %d", readTemperature(0));
    char string[20];
    while(1)
    {
        printf("Eingabe: ");
        scanf("%s", string);
        if(strstr(string, "Animation"))
        {
            printf("Senden: \n");
            if(sendWithRetry(2,"animation1,15",13,3,1000))
                printf("Ack Received\n");
            else
                printf("No Ack Received");
            //send(2,"animation1,15",14,1);
            printf("Node Adr: %d", readReg(0x39));
            printf("Net Adr: %d", readReg(0x30));
        }
    }
    return 0;
}