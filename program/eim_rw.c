#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define START_OFFSET 0
#define READ_TIMES (4*1024)

void ASCII2Hex(unsigned int *val, char *p_char)
{
    unsigned char l_cnt = 0;
    *val = 0;

    for (l_cnt = 0; l_cnt < 8; l_cnt++) {
        *val <<= 4;
        if (p_char[l_cnt] >= '0'&&p_char[l_cnt] <= '9') {
	    *val += p_char[l_cnt] - 0x30;
	} else {
	    *val += p_char[l_cnt] - 0x37;
	}
    }
}
int main(char argc, char **argv)
{
    unsigned int value = 0x55aaaa55;
    unsigned int res = 0;
    unsigned int l_cnt = 0;
    unsigned int l_times = 1000;
    int fd = 0;
    unsigned int offset = 0;
    unsigned short val_array[READ_TIMES] = {0};
    unsigned short *ptr = NULL;

    fd = open("/dev/eim_cs", O_RDWR);
    if (fd < 0) {
        printf("open file /dev/eim_cs failed!\n");
    }
#if 0
    lseek(fd, 0x20027c0, SEEK_SET);
    read(fd, (unsigned char *)(&res), 4);
    printf("current value: %X\n", res);
#endif   

    if (argc == 2) {
/*        value = (unsigned short)(*argv[1]-0x30);
        value <<= 4;
        value += *(argv[1]+1) - 0x30;
        value <<= 4;
        value += *(argv[1]+2) - 0x30;
        value <<= 4;
        value += *(argv[1]+3) - 0x30;
        value <<= 4;
        value += *(argv[1]+4) - 0x30;
        value <<= 4;
        value += *(argv[1]+5) - 0x30;
        value <<= 4;
        value += *(argv[1]+6) - 0x30;
        value <<= 4;
        value += *(argv[1]+7) - 0x30;

	ASCII2Hex(&offset, argv[1]);
*/
    }

#if 1
    value = 0xFBFFFBFF;
    lseek(fd, offset, SEEK_SET);
    write(fd, (unsigned char *)(&value), 4);
#endif
#if 1
    lseek(fd, offset, SEEK_SET);
    read(fd, (unsigned char *)val_array, READ_TIMES+START_OFFSET);
    //printf("0x%04X : %XH\n", offset, res);
    ptr = val_array + START_OFFSET;
    for (l_cnt = 0; l_cnt < READ_TIMES/2; l_cnt++) {
        printf("0x%04X ", ptr[l_cnt]);
        if (l_cnt%8 == 7) printf("\n");
    }
    printf("\n");
#endif
#if 0
    lseek(fd, 0x0, SEEK_SET);
    write(fd, (unsigned char *)(&value), 4);
    printf("write bytes to device finished!\n");
#endif
    //for (l_cnt=0; l_cnt<10000000; l_cnt++);

    close(fd);

    return 0;
}

