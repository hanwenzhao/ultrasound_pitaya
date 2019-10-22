#include <stdio.h>
#include <stdlib.h>
#include "redpitaya/rp.h"


int main(int argc, char **argv){
    
    /* Print error, if rp_Init() function failed */
    if(rp_Init() != RP_OK){
        fprintf(stderr, "Rp api init failed!\n");
    }
    int buff_size = 15000;
    float *x = (float *)malloc(buff_size * sizeof(float));
    for (int i = 0; i < buff_size; i++){
        x[i] = 0;
    }
    for (int i = 0; i < 10; i++){
        x[i] = 1;
    }
    for (int i = 20; i < 30; i++){
        x[i] = 1;
    }
    for (int i = 40; i < 50; i++){
        x[i] = 1;
    }
    for (int i = 60; i < 70; i++){
        x[i] = 1;
    }
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    rp_GenArbWaveform(RP_CH_2, x, buff_size);
    rp_GenFreq(RP_CH_2, 5000.0);
    rp_GenAmp(RP_CH_2, 1.0);
    rp_GenOutEnable(RP_CH_2);
    free(x);
    rp_Release();
}