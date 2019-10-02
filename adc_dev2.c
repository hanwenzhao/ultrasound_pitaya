#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "redpitaya/rp.h"

unsigned int changed_endian(unsigned int num){
    int byte0, byte1, byte2, byte3;
    byte0 = (num & 0x000000FF) >> 0 ;
    byte1 = (num & 0x0000FF00) >> 8 ;
    byte2 = (num & 0x00FF0000) >> 16 ;
    byte3 = (num & 0xFF000000) >> 24 ;
    return((byte0 << 24) | (byte1 << 16) | (byte2 << 8) | (byte3 << 0));
}

int main(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long time_stamp = tv.tv_usec;
    printf("%lu\n", time_stamp);

    time_stamp = changed_endian(time_stamp);

    unsigned char time_stamp_buffer[4];
    memcpy(time_stamp_buffer, (unsigned char *)&time_stamp, sizeof time_stamp_buffer);

    //unsigned char message_buffer[10];
    for (int i = 0 ; i < sizeof(time_stamp_buffer); i++){
        printf("%02X ",*(time_stamp_buffer + i));
    }
    printf("\n");

    return 0;
}