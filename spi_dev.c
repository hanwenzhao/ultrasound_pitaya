#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

int spi_fd = -1;
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

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
    uint8_t zero_set = 0x70;
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


int main(void){
    /* Init the spi resources */
    if(init_spi() < 0){
        fprintf(stderr, "Initialization of SPI failed. Error: %s\n", strerror(errno));
        return -1;
    }

    
	while(1){
		clock_t t;
		t = clock();
		float deg = read_encoder();
		fprintf(stdout, "%f\n", deg);
		t = clock() - t;
		double time_taken = ((double)t)/CLOCKS_PER_SEC;
		fprintf(stdout, "%f\n", time_taken);
	}
}