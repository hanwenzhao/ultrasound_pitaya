/* Red Pitaya C API example Generating continuous signal  
 * This application generates a specific signal */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "redpitaya/rp.h"

int main(int argc, char **argv){
	/* Print error, if rp_Init() function failed */
	if(rp_Init() != RP_OK){
		fprintf(stderr, "Rp api init failed!\n");
	}
	rp_GenReset();
	/* 5000 Hz Trigger Source at Output 1 */
	
	rp_GenFreq(RP_CH_1, 5000);
	rp_GenAmp(RP_CH_1, 1.0);
	rp_GenWaveform(RP_CH_1, RP_WAVEFORM_PWM);
	rp_GenOffset(RP_CH_1, 1.0);
	rp_GenDutyCycle(RP_CH_1, 0.0005);
	rp_GenOutEnable(RP_CH_1);
	
	/* 4.25MHz Chrip with 5 cycles and 1 ms burst period */
	/*
	rp_GenWaveform(RP_CH_2, RP_WAVEFORM_PWM);
	rp_GenFreq(RP_CH_2, 4250000);
	rp_GenAmp(RP_CH_2, 1.0);
	rp_GenOffset(RP_CH_2, 1.0);
	rp_GenDutyCycle(RP_CH_2, 0.1);
	rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
	rp_GenBurstCount(RP_CH_2, 3);
	rp_GenBurstRepetitions(RP_CH_2, 1);
	rp_GenBurstPeriod(RP_CH_2, 1);
	//rp_GenTriggerSource(RP_CH_2, RP_GEN_TRIG_SRC_EXT_PE);
	rp_GenTrigger(2);
	rp_GenOutEnable(RP_CH_2);
	*/
	/* Releasing resources */
	rp_Release();

	return 0;
}
