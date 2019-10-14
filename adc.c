#include <stdio.h>
#include <sys/time.h>
#include <time.h>
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

/* ##################### OPTIONS ##################### */
#define TXTFILE
//#define TCPIP

/* ##################### Defines ##################### */
/* ADC Defines */
#define TX_FREQ 5000.0
#define TX_AMP 1.0
#define TX_OFFSET 1.0
#define TX_DUTYCYCLE 0.1/(1000000/TX_FREQ) // 0.1us(100ns) / 200us
#define ADC_TRIG_LEVEL 1.8
#define ADC_DECIMATION RP_DEC_8
#define BUFF_SIZE 2490
#define ADC_TRIG_DELAY BUFF_SIZE-8192 // pitaya has a internal 8192 tigger delay
#define TRIG_SOURCE RP_TRIG_SRC_CHB_PE
#define TRIF_LEVEL_SOURCE RP_CH_2
#define BUFFER_FILL_TIME BUFF_SIZE/15.6 + 10 // need a little longer time than just about right

/* ##################### Varaibles ##################### */
/* SPI variables */
int spi_fd = -1;
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay;

/* main loop variables */
struct timeval tv;
rp_acq_trig_state_t state = RP_TRIG_STATE_WAITING;
char lo;
char hi;
uint32_t buff_size = BUFF_SIZE;
unsigned long time_stamp;
uint16_t encoder;
int16_t temp;
uint32_t crc_result;
const unsigned char marker[10] = {0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01};
unsigned char time_stamp_char[4];
unsigned char encoder_char[2];
unsigned char crc_char[4];
unsigned char adc_char[2*BUFF_SIZE];
unsigned char crc_input[4+2+2*BUFF_SIZE];
unsigned char message_buff[10+4+2+2*BUFF_SIZE+4];

/* ##################### Functions ##################### */
static int SPI_Init();
void TX_Init();
void ADC_Init();
void System_Init();
unsigned long changed_endian_4Bytes(unsigned long num);
int16_t changed_endian_2Bytes(int16_t value);
uint16_t read_encoder();
uint32_t rc_crc32(uint32_t crc, unsigned char *buf, size_t len);
static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len);
void print_bytes(unsigned char num[], int size);
int i,e;

/* ##################### TCP ##################### */
#define SERVER_IP "169.254.209.209"


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
    // initialize all systems
    System_Init();
    // prepare text file to write
    FILE * fp;
    fp = fopen("./test.txt", "w");
    // set trigger delay
    if (rp_AcqSetTriggerDelay((int32_t)ADC_TRIG_DELAY) != RP_OK){
        fprintf(stderr, "Error: Sets the number of decimated data after trigger written into memory failed!\n");
    }
    // main loop
    clock_t start, end;
    double cpu_time_used;
    start = clock();
    for (int j = 0; j < 1000; j++){
    //while(1){ 
        // start adc acquiring
        if(rp_AcqStart() != RP_OK){
            fprintf(stderr, "Error: Start of the Acquisition failed!\n");
        }
        // reset trigger state
        state = RP_TRIG_STATE_WAITING;
        // adc acquiring loop
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
                // time for adc to stop after trigger (2500-8192)
                //fprintf(stdout, "TRIG\n");
                usleep(BUFFER_FILL_TIME);
                break;
            }
            //fprintf(stdout, "Waiting for TRIG\n");
        }
        int16_t *buff = (int16_t *)malloc(buff_size * sizeof(int16_t));
        // get lastest data in raw
        if(rp_AcqGetLatestDataRaw(RP_CH_1, &buff_size, buff) != RP_OK){
            fprintf(stderr, "Returns the ADC buffer in Volt units from the oldest sample to the newest one failed!\n");
        }
        /* ##################### Time Stamp ##################### */
        // get the timestamp
        gettimeofday(&tv, NULL);
        // put microsecond timestamp into unsigned long
        time_stamp = tv.tv_usec;
        //printf("Time stamp is %lu\n", time_stamp);
        // convert small endian to big endian
        time_stamp = changed_endian_4Bytes(time_stamp);
        // put number in bytes
        memcpy(time_stamp_char, (unsigned char *)&time_stamp, sizeof time_stamp_char);
        //printf("Time stamp in bytes:\n");
        //print_bytes(time_stamp_char, sizeof time_stamp_char);
        /* ##################### Encoder ##################### */
        // read encoder
        encoder = read_encoder();
        //printf("Encoder is %d\n", encoder);
        // convert small endian to big endian
        encoder = changed_endian_2Bytes(encoder);
        // put number in bytes
        memcpy(encoder_char, &encoder, sizeof encoder_char);
        //printf("Time stamp in bytes:\n");
        //print_bytes(encoder_char, sizeof encoder_char);
        // convert all adc reading to big endian and put into unsigned char
        for (i = 0; i < BUFF_SIZE; i++){
            temp = buff[i];
            temp = changed_endian_2Bytes(temp);
            memcpy(adc_char+i*sizeof(temp), &temp, sizeof temp);
        }
        // concatenation everything we have now for crc calculation
        memcpy(crc_input, time_stamp_char, sizeof(time_stamp_char));
        memcpy(crc_input+sizeof(time_stamp_char), encoder_char, sizeof(encoder_char));
        memcpy(crc_input+sizeof(time_stamp_char)+sizeof(encoder_char), adc_char, sizeof(adc_char));
        // calculate crc32 checksum 
        crc_result = rc_crc32(0, crc_input, sizeof(crc_input));
        //printf("CRC is %X\n", crc_result);
        // change crc result to big endian
        crc_result = changed_endian_4Bytes(crc_result);
        // convert to unsigned char
        memcpy(crc_char, (unsigned char *)&crc_result, sizeof crc_char);
        //printf("CRC in bytes:\n");
        // put everything into message buffer
        memcpy(message_buff, marker, sizeof(marker));
        memcpy(message_buff+sizeof(marker), crc_input, sizeof(crc_input));
        memcpy(message_buff+sizeof(marker)+sizeof(crc_input), crc_char, sizeof(crc_char));
        #ifdef TXTFILE
            fwrite(&message_buff, sizeof(message_buff), 1, fp);
        #endif
        #ifdef TCPIP
        if ((e = write(sockfd, &message_buff, sizeof(message_buff))) < 0){
                fprintf(stdout, "ERROR writing to socket");
            }
        #endif
    }
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("%f seconds to execute \n", cpu_time_used);
    printf("%f Hz\n", 1.0/(cpu_time_used/1000));
}

int16_t changed_endian_2Bytes(int16_t value){
    return ((value >> 8) & 0x00ff) | ((value & 0x00ff) << 8);
}

void print_bytes(unsigned char num[], int size){
    for (int i = 0 ; i < size; i++){
            printf("%02X ",*(num + i));
    }
    printf("\n");
}

unsigned long changed_endian_4Bytes(unsigned long num){
    int byte0, byte1, byte2, byte3;
    byte0 = (num & 0x000000FF) >> 0 ;
    byte1 = (num & 0x0000FF00) >> 8 ;
    byte2 = (num & 0x00FF0000) >> 16 ;
    byte3 = (num & 0xFF000000) >> 24 ;
    return((byte0 << 24) | (byte1 << 16) | (byte2 << 8) | (byte3 << 0));
}

static int SPI_Init(){
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
    rp_acq_decimation_t rp_dec = ADC_DECIMATION;
    // set the decimation
    if (rp_AcqSetDecimation(rp_dec) != RP_OK){
        fprintf(stderr, "Error: Sets the decimation used at acquiring signal failed!\n");
    }
    // set trigger threshold
	if (rp_AcqSetTriggerLevel(TRIF_LEVEL_SOURCE, ADC_TRIG_LEVEL) != RP_OK){
        fprintf(stderr, "Error: Sets the trigger threshold value in volts failed!\n");
    }
}

void System_Init(){
    // check red pitaya api initialization
    if (rp_Init() != RP_OK){
        fprintf(stderr, "RP api init failed!\n");
    }
    // start generating Tx signal
    TX_Init();
    // initialize ADC
    ADC_Init();
    /* Init the spi resources */
    if(SPI_Init() < 0){
        fprintf(stderr, "Initialization of SPI failed. Error: %s\n", strerror(errno));
    }
}

uint32_t rc_crc32(uint32_t crc, unsigned char *buf, size_t len){
	static uint32_t table[256];
	static int have_table = 0;
	uint32_t rem;
	uint8_t octet;
	int i, j;
	unsigned char *p, *q;
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

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len){
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

uint16_t read_encoder(){
	uint8_t rd_pos = 0x10;
    uint8_t nop_a5 = 0x00;
    //uint8_t zero_set = 0x70;
    uint8_t rx_buff = 0xA5;
	uint8_t temp[2]; // temp variale to store MSB and LSB
	uint16_t ABSposition = 0;
	//float deg = 0.0;
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
	//deg = ABSposition * 360.0 / 4096.0;
	// wait a bit
	usleep(1);
	return ABSposition;	
}