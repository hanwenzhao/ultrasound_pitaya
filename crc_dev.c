#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
#include <zlib.h>
#include "redpitaya/rp.h"
#include <string.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
 
uint32_t rc_crc32(uint32_t crc, const char *buf, size_t len){
	static uint32_t table[256];
	static int have_table = 0;
	uint32_t rem;
	uint8_t octet;
	int i, j;
	const char *p, *q;
 
	/* This check is not thread safe; there is no mutex. */
	if (have_table == 0) {
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
 
int
main()
{
	//FILE *fp;
	//fp = fopen("./test2.txt", "w");
	int16_t buffer[10] = {-1, 10, 100, 1000, 10000, 9, 99, 999, 9999, 0};
	int i;
	char str[5];
	char dest[100];
	for (i = 0; i < 10; i++){
		sprintf(str, "%0X", buffer[i]);
		strcat(dest, "0x");
		strcat(dest, str);
		fprintf(stdout, "%s\n", dest);
	}
	//uint32_t x = rc_crc32(0, buff, strlen(buff));
	//fprintf(stdout, "%X\n", x);
 
	return 0;
}