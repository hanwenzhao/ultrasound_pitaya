#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <errno.h>
#include <arpa/inet.h> 
#include "redpitaya/rp.h"
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <memory.h>
#include <sys/ioctl.h>

// ************* TX and ADC *************
#define TX_FREQ 5000.0
#define TX_AMP 1.0
#define TX_OFFSET 1.0
#define TX_DUTYCYCLE 0.1/(1000000/TX_FREQ) // 0.1us(100ns) / 200us
#define ADC_TRIG_LEVEL 1.5
#define ADC_DECIMATION 1
#define BUFF_SIZE 2500
#define ADC_TRIG_DELAY BUFF_SIZE-8192 // pitaya has a internal 8192 tigger delay
#define COMMUNICATION_BUFFER_SIZE BUFF_SIZE*6
#define TRIG_SOURCE RP_TRIG_SRC_CHB_PE
#define TRIF_LEVEL_SOURCE RP_CH_2
#define BUFFER_FILL_TIME 2*BUFF_SIZE/125 // need a little longer time than just about right

int i, e;
int result;
uint32_t crc32_result;
char timestamp_buffer[20];
char crc32_buffer[20];
char data_buffer[COMMUNICATION_BUFFER_SIZE];
char encoder_buffer[20];
char message_buffer[COMMUNICATION_BUFFER_SIZE];



// ********** TCP ***************
#define SERVER_IP "169.254.224.122"

// ************* SPI ***********
int spi_fd = -1;
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;
float deg;


// option of how to transmit data
#define TXTFILE
//#define TCPIP

// option for output data
#define TIMESTAMP
#define CHECKSUM
#define ENCODER

// function prototype
void TX_Init();
void ADC_Init();
uint32_t rc_crc32(uint32_t crc, const char *buf, size_t len);
static int init_spi();
static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len);
float read_encoder();

int main(int argc, char **arg){
    #ifdef TCPIP
        // prepare for TCP communication
        int sockfd, portno = 51717, n;
        char serverIp[] = SERVER_IP;
        struct sockaddr_in serv_addr;
        struct hostent *server;
        fprintf(stdout, "contacting %s on port %d\n", serverIp, portno);
        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
            printf( "ERROR opening socket" );
        }
        if ((server = gethostbyname( serverIp)) == NULL){
            printf( "ERROR, no such host\n");
        }
        bzero( (char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy( (char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
        serv_addr.sin_port = htons(portno);
        if ( connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) 
            printf( "ERROR connecting");
    #endif

    /* Init the spi resources */
    if(init_spi() < 0){
        fprintf(stderr, "Initialization of SPI failed. Error: %s\n", strerror(errno));
        return -1;
    }

    // prepare text file to write
    FILE * fp;
    fp = fopen("./test13.txt", "w");
    // initilize time struct for timestamp
    #ifdef TIMESTAMP
        struct timeval tv;
    #endif
    // check api initialization
    if (rp_Init() != RP_OK){
        fprintf(stderr, "RP api init failed!\n");
    }
    // start generating Tx signal
    TX_Init();
    // initialize ADC
    ADC_Init();

    // create buff for adc reading
    uint32_t buff_size = BUFF_SIZE;
    int16_t *buff = (int16_t *)malloc(buff_size * sizeof(int16_t));

    
    // set trigger delay
    if (rp_AcqSetTriggerDelay((int32_t)ADC_TRIG_DELAY) != RP_OK){
        fprintf(stderr, "Error: Sets the number of decimated data after trigger written into memory failed!\n");
    }
    
    rp_acq_trig_state_t state = RP_TRIG_STATE_WAITING;
    
    // start loop
    while(1){
        //char * message_buffer = (char *) malloc(100 + sizeof(crc32_buffer)+ sizeof(data_buffer));
        // start adc acquiring
        if(rp_AcqStart() != RP_OK){
            fprintf(stderr, "Error: Start of the Acquisition failed!\n");
        }
        // reset trigger state
        state = RP_TRIG_STATE_WAITING;
        while(1){
            // set trigger source
            if(rp_AcqSetTriggerSrc(TRIG_SOURCE) != RP_OK){
                fprintf(stderr, "Error: Set of the adquisition trigger source failed!\n");
            }
            // get trigger state
            if (rp_AcqGetTriggerState(&state) != RP_OK){
                fprintf(stderr, "Error: Returns the trigger state failed!\n");
            }
            if (state == RP_TRIG_STATE_TRIGGERED){
                // time for adc to stop after trigger (8192+3000)
                fprintf(stdout, "TRIG\n");
                usleep(BUFFER_FILL_TIME);
                break;
            }
            //fprintf(stdout, "Waiting for TRIG\n");
        }
        if(rp_AcqGetLatestDataRaw(RP_CH_1, &buff_size, buff) != RP_OK){
            fprintf(stderr, "Returns the ADC buffer in Volt units from the oldest sample to the newest one failed!\n");
        }
        // get current time
        #ifdef TIMESTAMP
            gettimeofday(&tv, NULL);
            sprintf(timestamp_buffer, "%ld %ld ", tv.tv_sec, tv.tv_usec);
            strncat(data_buffer, timestamp_buffer, sizeof(timestamp_buffer));
        #endif

        // get encoder value
        #ifdef ENCODER
            deg = read_encoder();
            sprintf(encoder_buffer, "%f ", deg);
            strncat(data_buffer, encoder_buffer, sizeof(encoder_buffer));
        #endif

        char num[5];
		for (i = 0; i < buff_size; i++){
            // convert integer to char array
            sprintf(num, "%d", buff[i]);
            // combine each number to full output char array
            strncat(data_buffer, num, sizeof(num));
            // add space the seperate
            strncat(data_buffer, " ", 1);
		}
        
        //fprintf(stdout, "%s", data_buffer);
        #ifdef CHECKSUM
            // calculate the check sum
            //crc32_result = rc_crc32(0, data_buffer, strlen(data_buffer));
            sprintf(crc32_buffer, "%X ", crc32_result);
            strncat(message_buffer, crc32_buffer, sizeof(crc32_buffer));
        #endif

        // combine data into message buffer
        strncat(message_buffer, data_buffer, sizeof(data_buffer));
        strncat(message_buffer, "\n", 1);
        
        #ifdef TXTFILE
            // write data into file
            //fprintf(stdout, "%s", message_buffer);
            result = fputs(message_buffer, fp);
            if (result == EOF) {
                fprintf(stderr, "Error: Write data to file failed!\n");
            }
        #endif

        #ifdef TCPIP
            if ((e = write(sockfd, message_buffer, strlen(message_buffer))) < 0){
                fprintf(stdout, "ERROR writing to socket");
            }
        #endif
        // empty buffer
        memset(data_buffer, 0, sizeof data_buffer);
        memset(timestamp_buffer, 0, sizeof timestamp_buffer);
        memset(crc32_buffer, 0, sizeof crc32_buffer);
        memset(message_buffer, 0, sizeof message_buffer);
        //free(message_buffer);
    }
    fclose(fp);
	free(buff);
    rp_Release();
    #ifdef TCPIP
        close(sockfd);
    #endif
	return 0;
}


uint32_t rc_crc32(uint32_t crc, const char *buf, size_t len){
	static uint32_t table[256];
	static int have_table = 0;
	uint32_t rem;
	uint8_t octet;
	int i, j;
	const char *p, *q;
	/* This check is not thread safe; there is no mutex. */
	if (have_table == 0) {
        //fprintf(stdout, "Table\n");
		/* Calculate CRC table. */
		for (i = 0; i < 256; i++) {
			rem = i;  /* remainder from polynomial division */
			for (j = 0; j < 8; j++) {
				if (rem & 1) {
					rem >>= 1;
					rem ^= 0xedb88320;
				} else
					rem >>= 1;
			}
			table[i] = rem;
		}
		have_table = 1;
	}
	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc >> 8) ^ table[(crc & 0xff) ^ octet];
	}
	return ~crc;
}

void TX_Init(){
    // Generate Tx signal
	rp_GenFreq(RP_CH_1, TX_FREQ);
	rp_GenAmp(RP_CH_1, TX_AMP);
	rp_GenWaveform(RP_CH_1, RP_WAVEFORM_PWM);
	rp_GenOffset(RP_CH_1, TX_OFFSET);
	rp_GenDutyCycle(RP_CH_1, TX_DUTYCYCLE);
	rp_GenOutEnable(RP_CH_1);
}

void ADC_Init(){
    // rest adc
    if (rp_AcqReset() != RP_OK){
        fprintf(stderr, "Error: Resets the acquire writing state machine failed!\n");
    }
    // set the decimation
    if (rp_AcqSetDecimation(ADC_DECIMATION) != RP_OK){
        fprintf(stderr, "Error: Sets the decimation used at acquiring signal failed!\n");
    }
    // set trigger threshold
	if (rp_AcqSetTriggerLevel(TRIF_LEVEL_SOURCE, ADC_TRIG_LEVEL) != RP_OK){
        fprintf(stderr, "Error: Sets the trigger threshold value in volts failed!\n");
    }
}

static int init_spi(){
    int mode = 0;

    /* Opening file stream */
    // RDWR means it can read and write
    spi_fd = open("/dev/spidev1.0", O_RDWR | O_NOCTTY);

    if(spi_fd < 0){
        fprintf(stderr, "Error opening spidev0.1. Error: %s\n", strerror(errno));
        return -1;
    }

    /* Setting mode (CPHA, CPOL) */
    if(ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0){
        fprintf(stderr, "Error setting SPI_IOC_RD_MODE. Error: %s\n", strerror(errno));
        return -1;
    }

    /* Setting SPI bus speed */
    int spi_speed = 1000000;

    if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0){
        fprintf(stderr, "Error setting SPI_IOC_WR_MAX_SPEED_HZ. Error: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}


static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}
    
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		fprintf(stderr, "can't send spi message\n");
    
}

float read_encoder(){
	uint8_t rd_pos = 0x10;
    uint8_t nop_a5 = 0x00;
    //uint8_t zero_set = 0x70;
    uint8_t rx_buff = 0xA5;
	uint8_t temp[2]; // temp variale to store MSB and LSB
	uint16_t ABSposition = 0;
	float deg = 0.0;

	// send the first rd_pos command
	transfer(spi_fd, &rd_pos, &rx_buff, 1);
	// if the encoder is not ready, continue to send 0x00 command until it's ready
	while (rx_buff != rd_pos){
		//fprintf(stdout, "%02X\n", rx_buff);
		transfer(spi_fd, &nop_a5, &rx_buff, 1);
		usleep(1);
	}

	// get the MSB
	transfer(spi_fd, &nop_a5, &rx_buff, 1);
	temp[0] = rx_buff;
	// get LSB
	transfer(spi_fd, &nop_a5, &rx_buff, 1);
	temp[1] = rx_buff;

	// calculate the abs position
	// mask out first 4 bits
	temp[0] &= ~ 0xF0; 
	// shift MSB to ABS position
	ABSposition = temp[0] << 8;
	// add LSB
	ABSposition += temp[1];
	// convert to angle
	deg = ABSposition * 360.0 / 4096.0;
	
	// wait a bit
	usleep(1);
	
	return deg;	
}