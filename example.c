#include "RFM69.h"
#include <stdio.h>

#define FREQUENCY     RF69_868MHZ

int main(void)
{
    initialize(FREQUENCY, 1, 100, 16, 22, 0);
    readTemperature(0);
    char string[20];
    while(1)
    {
        printf("Eingabe: ");
        scanf("%s", string);
        if(strstr(string, "Animation"))
        {
            if(sendWithRetry(2,"animation1,15",13,3,10))
                printf("Ack Received");
            else
                printf("No Ack Received");
        }
    }
    return 0;
}