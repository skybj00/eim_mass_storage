#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/serial.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#define TRACE(fmt,args...) printf(fmt, ##args)

#define UART_1_FILE     "/dev/ttyUSB0"
#define ATTI_FRAME_HEADER_LEN     (4)
#define ATTI_FRAME_SIZE     (12)
#define ATTI_BUF_SIZE       (ATTI_FRAME_SIZE*4)

struct UpdateInfo {
    unsigned int sec;
    unsigned int usec;
    unsigned short deepth;
    unsigned short angle_x;
    short angle_y;
    short angle_z;
}fpga_reg;

static int searchOneFrame(unsigned char *buf,
        unsigned char len,
        unsigned char *header,
        unsigned char header_len,
        unsigned char frame_len)
{
    unsigned int i,j;

    for (i=0; i<len-header_len; i++) {
        for (j=0; j<header_len; j++) {
            if (buf[i+j] != header[j]) break;
        }
        if (j == header_len) break;
    }
    if (i <= (len - frame_len)) return i;
    else return -1;
}

static void getBroadcastAttitudeFromUart(void)
{
    int fd;
    struct termios option;
    struct serial_struct serial;
    unsigned char frameHeader[ATTI_FRAME_HEADER_LEN] = {0xFF, 0x7E, 0x0C, 0x8A};
    unsigned char responseBuf[ATTI_BUF_SIZE] = {0};
    unsigned char l_read_cnt,l_cnt;
    int l_start_cnt;

    fd = open(UART_1_FILE, O_RDWR|O_NOCTTY);
    if (fd <= 0) {
        TRACE("%s open failed!\n", UART_1_FILE);
    } else {
        //TRACE("%s open succeed!\n", UART_CONFIG_FILE);
    }
#if 1
    /* serial configuration */
    ioctl(fd, TIOCGSERIAL, &serial);
    tcgetattr(fd, &option);
    /* set baud rate */
    if(cfsetispeed(&option, B38400)<0){
        TRACE("%s: set ispeed failed!\n", UART_1_FILE);
    }
    if(cfsetospeed(&option, B38400)<0){
        TRACE("%s: set ospeed failed!\n", UART_1_FILE);
    }
    if(cfsetspeed(&option, B38400)<0){
        TRACE("%s: set ospeed failed!\n", UART_1_FILE);
    }

    /* serial attributes set */
    option.c_cflag = CS8|CREAD|CLOCAL;
    option.c_iflag = 0;
    option.c_iflag &= ~BRKINT;
    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = ATTI_FRAME_SIZE*2+3;
    tcsetattr(fd, TCSANOW, &option);
#endif
    system("stty -F /dev/ttyUSB0 38400 \n");
    /* read response from device */
    memset(responseBuf, 0, sizeof(responseBuf));
    l_read_cnt = read(fd, responseBuf, ATTI_BUF_SIZE);
    l_start_cnt = searchOneFrame(responseBuf, l_read_cnt, frameHeader,
                ATTI_FRAME_HEADER_LEN, ATTI_FRAME_SIZE);
    if(l_start_cnt >= 0 && l_read_cnt > ATTI_FRAME_SIZE){
        /* print recieved frame info */
        TRACE("Attitude frame %d : \n", l_start_cnt);
        for(l_cnt=l_start_cnt; l_cnt<l_start_cnt+ATTI_FRAME_SIZE; l_cnt++){
            TRACE("%02X ", responseBuf[l_cnt]);
        }
        TRACE("\n");
        fpga_reg.angle_x = (unsigned short)(responseBuf[l_start_cnt+7]<<8)
                            +responseBuf[l_start_cnt+6];
        if((responseBuf[l_start_cnt+9]&0x80)==0x80){ 
            /* value is negative while MSB equals to 1 */
            fpga_reg.angle_y = -1*((unsigned short)((responseBuf[l_start_cnt+9]&0x7f)<<8)+
                                responseBuf[l_start_cnt+8]);
        } else {
            fpga_reg.angle_y = (unsigned short)((responseBuf[l_start_cnt+9]&0x7f)<<8)+
                                responseBuf[l_start_cnt+8];
        }
        if((responseBuf[l_start_cnt+11]&0x80)==0x80){
            fpga_reg.angle_z = -1*((unsigned short)((responseBuf[l_start_cnt+11]&0x7f)<<8)+
                                responseBuf[l_start_cnt+10]);
        } else {
            fpga_reg.angle_z = (unsigned short)((responseBuf[l_start_cnt+11]&0x7f)<<8)+
                                responseBuf[l_start_cnt+10];
        }
    }
}


int main()
{
    while (1) {
        getBroadcastAttitudeFromUart();
        printf("X: %d, Y: %d, Z: %d\n", 
                fpga_reg.angle_x, 
                fpga_reg.angle_y, 
                fpga_reg.angle_z);
        usleep(200000);
    }
}


