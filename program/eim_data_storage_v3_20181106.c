/*
 * COPYRIGHT (C) 2017-2018, XGD, 
 * All rights reserved.
 *
 * write by william, at 6/29/2018
 */

/*
 * @file eim_speed_test.c
 * @brief EIM data read and storage test program
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>   /* used for time get and time set */
#include <sys/ioctl.h>
#include <signal.h>
#include <errno.h>
/* header for serial */
#include <linux/serial.h>
#include <termios.h>
// include header for opendir
#include <sys/types.h>
#include <dirent.h>

#include <pthread.h>
#include <semaphore.h>

/*****************************************************************************/
/*************************** Macros ******************************************/
#define USE_SYSTEM_WR
//#define USER_FILE_FLUSH
#define USE_FILE_DATE
#define DEBUG_ENABLE	1
#if DEBUG_ENABLE
#define TRACE(fmt, args...) printf(fmt,##args)
#else
#define TRACE(fmt, args...)
#endif

#define READ_BUF_SIZE       (64*1024)
#define BLOCK_NUM           (2048)
#define MAIN_FILE_NAME      "DATA_"
#define STORAGE_PATH        "/mnt/ssd/"
#define FILE_INDEX_LEN      (3)
#define FILE_DATE_LEN       (12)
#define SECTOR_COUNT        (8192)
#define FFLUSH_COUNT        (20)
#define FILE_HEADER_LEN     (12)
#define CST_DIFF_IN_SEC     (8*3600)
#define DEVICE_FILE_NAME    "/dev/eim_cs"
#define DATA_READY_FILE     "/dev/data_ready"
#define UART_CONFIG_FILE    "/dev/ttymxc4"
#define UART_1_FILE         "/dev/ttyUSB1"
#define UART_2_FILE         "/dev/ttyUSB0"
#define GPIO_SPI_FILE       "/dev/gpio_spi"

#define ATTI_FRAME_HEADER_LEN     (4)
#define ATTI_FRAME_SIZE     (12)
#define ATTI_VALUE_MULTIPLY     (4)
#define ATTI_BUF_SIZE       (ATTI_FRAME_SIZE*4)
/* A entire attitude frame contains 12 bytes, the structure is as
 * 0xFF 7E 0C 8A XL XH AL AH PL PH RL RH
 * {XH, XL} refers to frame count, a unsigned short value.
 * {AH, AL} refers to heading value, u16-le, range from 0 to 6400.
 * {PH, PL} refers to tilt value, s16-le, negetive while D15 == 1.
 * {RL, RH} refers to roll value, s16-le, negetive while D15 == 1.
 */
#define ATTI_FRAME_AL       (6)
#define ATTI_FRAME_AH       (7)
#define ATTI_FRAME_PL       (8)
#define ATTI_FRAME_PH       (9)
#define ATTI_FRAME_RL       (10)
#define ATTI_FRAME_RH       (11)
#define ATTI_BOUND_X_HIGH   ((unsigned short)(6400*ATTI_VALUE_MULTIPLY))
#define ATTI_BOUND_X_LOW    ((unsigned short)(0*ATTI_VALUE_MULTIPLY))
#define ATTI_BOUND_Y_HIGH   ((short)((90.0/360)*6400*ATTI_VALUE_MULTIPLY))
#define ATTI_BOUND_Y_LOW    ((short)((-90.0/360)*6400*ATTI_VALUE_MULTIPLY))
#define ATTI_BOUND_Z_HIGH   ((short)((90.0/360)*6400*ATTI_VALUE_MULTIPLY))
#define ATTI_BOUND_Z_LOW    ((short)((-90.0/360)*6400*ATTI_VALUE_MULTIPLY))

#define PERIODIC_FUNC_PERIOD_USEC   (200000)    /* set period to 200 ms */
/*****************************************************************************/
/*****************************Global Variable*********************************/
struct databuf{
    unsigned char dataBuf[READ_BUF_SIZE*BLOCK_NUM];
    unsigned int g_WtId;
    unsigned int g_RdId;
    unsigned int size;
}buf;
struct UpdateInfo {
    unsigned int sec;
    unsigned int usec;
    unsigned short deepth;
    unsigned short angle_x;
    short angle_y;
    short angle_z;
}fpga_reg;
//unsigned char readBuf[READ_BUF_SIZE] = {0};
sem_t sem_notify;
sem_t sem_complete;
unsigned int g_sector_cnt = 0;
char storage_file_name[64] = {0};
//just for test 
volatile unsigned long g_state_cnt = 0;
volatile unsigned long g_state_valid = 0;

/* use mutex to pretect databuf struct */
pthread_mutex_t mutex;
/* pthread variables */
pthread_t pth_read, pth_write, pth_detect,pth_comm;
/* file descriptor */
int fd_detect,fd_read = 0;
#ifdef USE_SYSTEM_WR
    int fd_write;
#else
    FILE* flip_write;
#endif
/* global variables */
unsigned char g_recording_flag = 0;
unsigned char g_recording_flag_prev = 0;
unsigned char g_program_exit_flag = 0;
unsigned int g_sample_rate_in_hz = 8000;
/****************************************************************************/
/*****************************Private function Declartion********************/
static __sighandler_t program_exit(void);
static __sighandler_t periodic_func(void);
static void genNewFileName(char *name);
static void convertToBrokenDownTime(struct tm *tm_out, unsigned char *buf);
static void writeDataToSpi(unsigned int val);
static void writeSampleRateToSpi(unsigned char val);
static void writeRecordCtrlToSpi(unsigned char val);
static void periodic_func_setting(void);
static void setSysTime(unsigned char *buf);
static void writeDataToSpi(unsigned int val);
static void getDeepthFromUart(void);
static void getAttitudeFromUart(void);
static void getBroadcastAttitudeFromUart(void);

/****************************************************************************/
/*****************************function definition****************************/
/*
 * @brief   read data_ready signal and pull semaphore
 */
void *signal_detect(void *arg)
{
    unsigned char data_status = 0;

    fd_detect = open(DATA_READY_FILE, O_RDWR);
    if (fd_detect <= 0) {
        TRACE("%s open failed!\n", DATA_READY_FILE);
    } else {
        TRACE("%s open succeed!\n", DATA_READY_FILE);
    }
    
    while(1){
        sem_wait(&sem_complete);
        // block p2rogram while buffer is full
        while(buf.size >= (READ_BUF_SIZE*BLOCK_NUM));
        // read() will NOT return until data is ready for reading
        do {
            read(fd_detect, &data_status, 1); 
            /* g_state_cnt++; */
        }while(data_status==0);
        /* g_state_valid++; */
        sem_post(&sem_notify);
        if (!g_recording_flag) {
            TRACE("pthread: signal_detect exit!\n");
            close(fd_detect);
            pthread_exit(0);
        }
    }
}
/*
 * @brief  read pthread of program, getting data from eim port
 */
void *data_read(void *arg)
{
    /* open EIM device file */
    fd_read = open(DEVICE_FILE_NAME, O_RDONLY);
    if (fd_read == 0) {
        TRACE("%s open failed!\n", DEVICE_FILE_NAME);
    } else {
        TRACE("%s open finished!\n", DEVICE_FILE_NAME);
    }

    while (1) {
        sem_wait(&sem_notify);

        read(fd_read, &(buf.dataBuf[buf.g_WtId]), READ_BUF_SIZE);
        
        pthread_mutex_lock(&mutex);
        buf.g_WtId += READ_BUF_SIZE;
        buf.size += READ_BUF_SIZE;
        if (buf.g_WtId >= (READ_BUF_SIZE*BLOCK_NUM)) {
            buf.g_WtId = 0;
        }
        pthread_mutex_unlock(&mutex);

        sem_post(&sem_complete);

        if (!g_recording_flag) {
            TRACE("pthread: data_read exit!\n");
            close(fd_read);
            pthread_exit(0);
        }
    }
}

/*
 * @brief   write pthread of program, writing data to binary file
 */
void *data_write(void *arg)
{

    static unsigned int flush_cnt = 0;

#ifdef USE_SYSTEM_WR
    fd_write = open(storage_file_name, 
                O_WRONLY|O_SYNC);
                //|S_IRUSER|S_IWUSER|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
    if (fd_write <= 0) {
        TRACE("%s open failed!\n", storage_file_name);
    } else {
        TRACE("%s open finished!\n", storage_file_name);
    }
    lseek(fd_write, FILE_HEADER_LEN, SEEK_SET); 
#else
    flip_write = fopen(storage_file_name, "wb+");
    if (flip_write == NULL) {
        TRACE("%s fopen failed!\n", storage_file_name);
    } else {
        TRACE("%s fopen finished!\n", storage_file_name);
    }
#endif

    while (1) {
        /* block the program while size is less than READ_BUF_SIZE */
        while(buf.size < READ_BUF_SIZE);
#ifdef USE_SYSTEM_WR
        /* write data to file, using write() */
        write(fd_write, &(buf.dataBuf[buf.g_RdId]), READ_BUF_SIZE);
#ifdef USER_FILE_FLUSH
        if (flush_cnt++ >= FFLUSH_COUNT) {
            flush_cnt = 0;
            syncfs(fd_write);
        }
#endif
#else
        /* write data to file, using fwrite() */
        fwrite(&(buf.dataBuf[buf.g_RdId]), 1, READ_BUF_SIZE, flip_write);
#ifdef USER_FILE_FLUSH
        /* flush data in file buffer to physical sectors */
        if (flush_cnt++ >= FFLUSH_COUNT) {
            flush_cnt = 0;
            fflush(flip_write);
        }
#endif
#endif
        pthread_mutex_lock(&mutex);
        buf.g_RdId += READ_BUF_SIZE;
        buf.size -= READ_BUF_SIZE;
        if (buf.g_RdId >= (READ_BUF_SIZE*BLOCK_NUM)) {
            buf.g_RdId = 0;
        }
        pthread_mutex_unlock(&mutex);
        /* pthread stop judgement */
        if (!g_recording_flag) {
#ifdef USE_SYSTEM_WR
            close(fd_write);
#else
            fclose(flip_write);
#endif
            TRACE("pthread: data_write exit!\n");
            pthread_exit(0);
        }
    }
}

/*
 * @brief   uart communication pthread for UART_CONFIG
 */
void *uart_comm(void *arg)
{
    int fd;
    unsigned char recvbuf[16] = {0};
    unsigned char sendbuf[8] = {0};
    unsigned int time_second_cnt = 0;
    unsigned char i,sum,rd_cnt;
    const int uart_comm_frame_size = 10;
    const int frame_check_offset = 9;
    const int frame_cmd_offset = 1;
    struct termios option;
    struct serial_struct serial;

    fd = open(UART_CONFIG_FILE, O_RDWR|O_NOCTTY);
    if (fd <= 0) {
        TRACE("%s open failed!\n", UART_CONFIG_FILE);
    } else {
        //TRACE("%s open succeed!\n", UART_CONFIG_FILE);
    }
#if 1
    /* serial configuration */
    ioctl(fd, TIOCGSERIAL, &serial);
    tcgetattr(fd, &option);
    /* set baud rate */
    if(cfsetispeed(&option, B9600)<0){
        TRACE("set ispeed failed!\n");
    }
    if(cfsetospeed(&option, B9600)<0){
        TRACE("set ospeed failed!\n");
    }
    /* serial attributes set */
    option.c_cflag = CS8|CREAD|CLOCAL;
    option.c_iflag = 0;
    option.c_iflag &= ~BRKINT;
    option.c_iflag |= IGNBRK;
    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_cc[VTIME] = 1;
    option.c_cc[VMIN] = 10;
    tcsetattr(fd, TCSANOW, &option);
#endif
    /* send system start message to host*/
    sendbuf[0] = 0xFF;
    sendbuf[1] = 0x00;
    sendbuf[2] = 0x5a;
    write(fd, sendbuf, 3);

    while(1){
        /* clear recieve buffer */
        memset(recvbuf, 0, 16);
        /* read communication frame */
        rd_cnt = read(fd, recvbuf, uart_comm_frame_size);
        /* check the frame via add-up checking */
        sum = 0;
        for(i=frame_cmd_offset; i<uart_comm_frame_size-1; i++){
            sum += recvbuf[i];
        }
        if((sum+recvbuf[frame_check_offset]) != 0xFF){
            if(rd_cnt>uart_comm_frame_size){
                TRACE("uart frame length exceed!\n");
                continue;
            } else {
                TRACE("uart sum check failed! cnt: %d, check: 0x%02X\n",rd_cnt, sum); 
                for(i=0; i<rd_cnt; i++){
                    TRACE("%02X ", recvbuf[i]);
                }
                printf("\n");
                continue;
            }
        }
        /* ananlyze the comm frame and take actions */
        if(recvbuf[frame_cmd_offset]==0x81){
            /* setting system time*/
            setSysTime(&recvbuf[frame_cmd_offset+1]);
            /* pass sample rate to device via SPI*/
            writeSampleRateToSpi(recvbuf[frame_check_offset-1]);
            /* update sample rate variable */
            g_sample_rate_in_hz = (unsigned int)(recvbuf[frame_check_offset-1]*1000);
            /* send feedback info */
            sendbuf[0] = 0xFF;
            sendbuf[1] = 0x01;
            sendbuf[2] = 0x00;
            write(fd, sendbuf, 3);            
        }else if(recvbuf[frame_cmd_offset]==0x82){
            /* change recording status */
            g_recording_flag = (recvbuf[frame_cmd_offset+1]==0x00);
            writeRecordCtrlToSpi(g_recording_flag);
            /* send feedback info */
            sendbuf[0] = 0xFF;
            sendbuf[1] = 0x02;
            sendbuf[2] = (g_recording_flag==0x00);
            write(fd, sendbuf, 3);
        }

        if(g_program_exit_flag){
            TRACE("pthread: uart_comm exit!\n");
            close(fd);
            pthread_exit(0);
        }
    }
}
/*****************************************************************************/
/*****************************************************************************/
/*
 * @name    program_exit
 * @brief   the program is exit by Ctrl-C while program is runnig in the front.
 */
static __sighandler_t program_exit(void)
{
    g_program_exit_flag = 1;
    TRACE("program exit finished!\n");
}
/*
 * @name    convertTimeToStr
 * @brief   convert time of struct tm format to self-defined string, such as 
 *          "_20181031_120018", which refers to 2018/10/31 12:00:18.  
 */
static void convertTimeToStr(char *str, struct tm *time)
{
    char str_tmp[8] = {0};

    strcpy(str, "_");
    sprintf(str_tmp, "%04d", time->tm_year + 1900);
    strcat(str, str_tmp);
    sprintf(str_tmp, "%02d", time->tm_mon + 1);
    strcat(str, str_tmp);
    sprintf(str_tmp, "%02d", time->tm_mday);
    strcat(str, str_tmp);
    strcat(str, "_");
    sprintf(str_tmp, "%02d", time->tm_hour);
    strcat(str, str_tmp);
    sprintf(str_tmp, "%02d", time->tm_min);
    strcat(str, str_tmp);
    sprintf(str_tmp, "%02d", time->tm_sec);
    strcat(str, str_tmp);
}
/*
 * @name genNewFileName
 * @brief generate new file name in preset path
 */
static void genNewFileName(char *name)
{
    DIR *dir;
    struct dirent *dirent;
    char char_index[4] ={0};
    unsigned char current_index = 0;
    char *index_start;
    char *index_stop;
#ifdef USE_FILE_DATE
    char str_date[16] = {0};
    time_t t_sec;
    struct tm *tm_now;
#endif

    dir = opendir(STORAGE_PATH);
    if (dir == NULL) {
        TRACE("open dir %s failed!\n", STORAGE_PATH);
    } else {
        TRACE("open dir %s succeed!\n", STORAGE_PATH);
    }
    while(1){
        dirent = readdir(dir);
        if (dirent == NULL) {
            break;
        }
        //TRACE("read dir succeed! cur_dir = %s\n", dirent->d_name);
        /* when . or .. is read, continue */
        if (dirent->d_name[0] == '.') {
            continue;
        }
        if (!memcmp(dirent->d_name, MAIN_FILE_NAME, strlen(MAIN_FILE_NAME))) {
            index_start = dirent->d_name + strlen(MAIN_FILE_NAME);
            /* index_stop = strchr(dirent->d_name, '.'); */
            memcpy(char_index, index_start, FILE_INDEX_LEN);
            char_index[FILE_INDEX_LEN + 1] = '\0';
            if (atoi(char_index) > current_index) {
                current_index = atoi(char_index);
            }
            //TRACE("current file index: %d\n", current_index);
        }
    }
    closedir(dir);

    // create full file name
    strcpy(name, STORAGE_PATH);
    strcat(name, MAIN_FILE_NAME);
    sprintf(char_index, "%03d", current_index+1);
    strcat(name, char_index);
#ifdef USE_FILE_DATE
    /* date string length is defined by FILE_DATE_LEN */
    memset(str_date, 0, sizeof(str_date));
    time(&t_sec);
    tm_now = localtime(&t_sec);
    convertTimeToStr(str_date, tm_now);
    strcat(name, str_date);
#endif
    strcat(name, ".dat");

    TRACE("new file name is : %s\n", name);
}
/*
 * @name    writeHeaderToFile
 * @brief   write seconds, microseconds, sample-rate to file in little-endian.
 */
static void writeHeaderToFile(char *name)
{
    int fd;
    time_t time_cur;
    unsigned char buffer[FILE_HEADER_LEN] = {0};
    unsigned char wr_cnt = 0;

    fd = open(name, O_RDWR|O_CREAT);
    if(fd < 0){
        TRACE("open %s failed!\n", name);
    }
    /* get time */
    time(&time_cur);
    *((unsigned int*)buffer) = (unsigned int)time_cur+CST_DIFF_IN_SEC;
    *((unsigned int*)&buffer[4]) = 0;
    /* get sample rate  */
    *((unsigned int*)&buffer[8]) = g_sample_rate_in_hz;
    /* write header to file */
    wr_cnt = write(fd, buffer, FILE_HEADER_LEN);
    if(wr_cnt != FILE_HEADER_LEN){
        TRACE("file header write failed! %d byte writen!\n", wr_cnt);
    } else {
        TRACE("file header write succeed!\n");
    }
    close(fd);
}
/*
 * @name    convertToBrokenDownTime
 * @brief   convert time value which is BCD format in the buffer to broken-down
 *          value.
 */
static void convertToBrokenDownTime(struct tm *tm_out, unsigned char *buf)
{
    /* num of seconds after minutes, in the range 0 to 59 */
    tm_out->tm_sec = (int)((buf[0]>>4)*10+(buf[0]&0x0f));
    /* num of minutes, in the range 0 to 59 */
    tm_out->tm_min = (int)((buf[1]>>4)*10+(buf[1]&0x0f));
    /* num of hours, in the range 0 to 23 */
    tm_out->tm_hour = (int)((buf[2]>>4)*10+(buf[2]&0x0f));
    /* day of the month, in the range 1 to 31 */
    tm_out->tm_mday = (int)((buf[3]>>4)*10+(buf[3]&0x0f));
    /* num of the months, in the range 0 to 11 */
    tm_out->tm_mon = (int)((buf[4]>>4)*10+(buf[4]&0x0f)-1);
    /* num of years since 1900 */
    tm_out->tm_year = (int)((buf[5]>>4)*10+(buf[5]&0x0f)+100);
}
/*
 * @name    writeDataToSpi
 * @brief   write 32-bits to spi
 */
static void writeDataToSpi(unsigned int val)
{
    int fd = 0;

    fd = open(GPIO_SPI_FILE, O_RDWR);
    if(fd<0){
        TRACE("open %s failed!\n", GPIO_SPI_FILE);
    }
    write(fd, (unsigned char*)&val, 4);

    close(fd);
}
/*
 * @name    writeSampleRateToSpi
 * @brief   write sample rate via spi to FPGA
 */
static void writeSampleRateToSpi(unsigned char val)
{
    writeDataToSpi(0x81000000 + val);
    TRACE("sample rate change to %d KHz\n", val);
}
/*
 * @name    writeRecordCtrlToSpi
 * @brief   write recording enable/disable via spi to FPGA
 */
static void writeRecordCtrlToSpi(unsigned char val)
{
    writeDataToSpi(0x80000000 + val);
    TRACE("g_recording_flag change to %2X\n", val);
}
/*
 * @name    writeTimeToSpi
 * @brief   write system time to FPGA via spi
 */
static void writeSysTimeToSpi(struct UpdateInfo *reg)
{
    unsigned char reg_tmp[4] = {0};
    unsigned char *ptr;
    unsigned int reg_down = 0;

    ptr = (unsigned char*)(&(reg->sec));
    reg_tmp[3]=0x82;
    reg_tmp[2]=ptr[3];
    reg_tmp[1]=ptr[2];
    reg_tmp[0]=ptr[1];
    reg_down = *((unsigned int*)reg_tmp);
    writeDataToSpi(reg_down);
    //TRACE("send to fpga : 0x%08X\n", reg_down);
    
    reg_tmp[3]=0x83;
    reg_tmp[2]=ptr[0];
    ptr = (unsigned char*)(&(reg->usec));
    reg_tmp[1]=ptr[3];
    reg_tmp[0]=ptr[2];
    reg_down = *((unsigned int*)reg_tmp);
    writeDataToSpi(reg_down);
    //TRACE("send to fpga : 0x%08X\n", reg_down);
    
    reg_tmp[3]=0x84;
    reg_tmp[2]=0x00;
    reg_tmp[1]=ptr[1];
    reg_tmp[0]=ptr[0];
    reg_down = *((unsigned int*)reg_tmp);
    writeDataToSpi(reg_down);
    //TRACE("send to fpga : 0x%08X\n", reg_down);
}
/*
 * @name    writeDeepthToSpi
 * @brief   write system deepth info which store in unsigned short variable.
 */
static void writeDeepthToSpi(struct UpdateInfo *reg)
{
    writeDataToSpi(0x85000000+reg->deepth);
}
/*
 * @name    writeAttitudeToSpi
 * @brief   write system Attitude info which store in 3 unsigned short variable.
 */
static void writeAttitudeToSpi(struct UpdateInfo *reg)
{
    if (reg->angle_x>=ATTI_BOUND_X_LOW&&reg->angle_x<=ATTI_BOUND_X_HIGH) {
        writeDataToSpi(0x86000000 | (reg->angle_x & 0x0000ffff));
    }
    if (reg->angle_y>=ATTI_BOUND_Y_LOW&&reg->angle_y<=ATTI_BOUND_Y_HIGH) {
        writeDataToSpi(0x87000000 | (reg->angle_y & 0x0000ffff));
    }
    if (reg->angle_z>=ATTI_BOUND_Z_LOW&&reg->angle_z<=ATTI_BOUND_Z_HIGH) {
        writeDataToSpi(0x88000000 | (reg->angle_z & 0x0000ffff));
    }
    TRACE("SPI VALUE: 0x%08X, 0x%08X, 0x%08X\n", 
            0x86000000|(reg->angle_x & 0x0000ffff),
            0x87000000|(reg->angle_y & 0x0000ffff),
            0x88000000|(reg->angle_z & 0x0000ffff));
}
/*
 * @name    getDeepthFromUart
 * @brief   get deepth information from UART_1 device 
 */
static void getDeepthFromUart(void)
{

}
/*
 * @brief   find frame header which is specified in unsigned char *header.
 *          return -1 if no header is found.
 */
static int findHeader(unsigned char *buf,
        unsigned char len,
        unsigned char *header)
{
    unsigned int i,j;

    for (i=0; i<len-ATTI_FRAME_HEADER_LEN; i++) {
        for (j=0; j<ATTI_FRAME_HEADER_LEN; j++) {
            if (buf[i+j] != header[j]) break;
        }
        if (j == ATTI_FRAME_HEADER_LEN) break;
    }
    if (i < (len - ATTI_FRAME_HEADER_LEN)) return i;
    else return -1;
}
/*
 * @brief   search specified header from buffer, if no entire frame is found, 
 *          return -1; otherwise return the point of frame start.
 */
static int searchOneFrame(unsigned char *buf, 
        unsigned char len,
        unsigned char *header)
{
    unsigned int i,j;
    int frameStart,frameStart2;

    if (len < ATTI_FRAME_SIZE + ATTI_FRAME_HEADER_LEN) 
        return -1;
    frameStart = findHeader(buf, len, header);
    if (frameStart < 0) {
        TRACE("first header is not found\n");
        return -1;
    } else {
        if (frameStart > (len - ATTI_FRAME_SIZE - ATTI_FRAME_HEADER_LEN)){
            TRACE("no entire frame, len: %d, start: %d\n", len, frameStart);
            return -1;
        }
        frameStart2 = findHeader(&buf[frameStart+ATTI_FRAME_SIZE], 
                                len-frameStart-ATTI_FRAME_SIZE,
                                header);
        if (frameStart2 < 0) {
            TRACE("second header not found! len: %d, 1st start: %d\n",
                    len, frameStart);
            return -1;
        } else {
            return frameStart;
        }
    }
}
/*
 * @brief   get Attitude info from UART_2, device send message periodicly, 
 *          the program read double size of frame size, and find entire
 *          frame, get Attitude info.
 */
static void getBroadcastAttitudeFromUart(void)
{
    int fd;
    struct termios option;
    struct serial_struct serial;
    unsigned char frameHeader[ATTI_FRAME_HEADER_LEN] = {0xFF, 0x7E, 0x0C, 0x8A};
    unsigned char responseBuf[ATTI_BUF_SIZE] = {0};
    unsigned char frameBuf[ATTI_FRAME_SIZE] = {0};
    char sysCmd[64] = {0};
    unsigned char l_read_cnt,l_cnt;
    int l_start_cnt;

    fd = open(UART_2_FILE, O_RDWR|O_NOCTTY);
    if (fd <= 0) {
        TRACE("%s open failed!\n", UART_2_FILE);
    } else {
        //TRACE("%s open succeed!\n", UART_2_FILE);
    }
#if 1
    /* serial configuration */
    ioctl(fd, TIOCGSERIAL, &serial);
    tcgetattr(fd, &option);
    /* set baud rate */
    if(cfsetispeed(&option, B0)<0){
        TRACE("%s: set ispeed failed!\n", UART_2_FILE);
    }
    if(cfsetospeed(&option, B38400)<0){
        TRACE("%s: set ospeed failed!\n", UART_2_FILE);
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
    strcpy(sysCmd, "stty -F ");
    strcat(sysCmd, UART_2_FILE);
    strcat(sysCmd, " 38400\n");
    system(sysCmd);
    /* read response from device */
    memset(responseBuf, 0, sizeof(responseBuf));
    l_read_cnt = read(fd, responseBuf, ATTI_BUF_SIZE);
    l_start_cnt = searchOneFrame(responseBuf, l_read_cnt, frameHeader);
    if(l_start_cnt >= 0){
        memcpy(frameBuf, &responseBuf[l_start_cnt], ATTI_FRAME_SIZE);
        /* print recieved frame info */
        TRACE("Attitude frame, %d : ", l_start_cnt);
        for(l_cnt=0; l_cnt<ATTI_FRAME_SIZE; l_cnt++){
            TRACE("%02X ", frameBuf[l_cnt]);
        }
        TRACE("\n");
        fpga_reg.angle_x = (unsigned short)(frameBuf[ATTI_FRAME_AH]<<8)
                            +frameBuf[ATTI_FRAME_AL];
        if((frameBuf[ATTI_FRAME_PH]&0x80)==0x80){ 
            /* value is negative while MSB equals to 1 */
            fpga_reg.angle_y = (short)(-1*((unsigned short)
                ((frameBuf[ATTI_FRAME_PH]&0x7f)<<8)+frameBuf[ATTI_FRAME_PL]));
        } else {
            fpga_reg.angle_y = (unsigned short)((frameBuf[ATTI_FRAME_PH]&0x7f)<<8)+
                                frameBuf[ATTI_FRAME_PL];
        }
        if((frameBuf[ATTI_FRAME_RH]&0x80)==0x80){
            fpga_reg.angle_z = (short)(-1*((unsigned short)
                ((frameBuf[ATTI_FRAME_RH]&0x7f)<<8)+frameBuf[ATTI_FRAME_RL]));
        } else {
            fpga_reg.angle_z = (unsigned short)((frameBuf[ATTI_FRAME_RH]&0x7f)<<8)+
                                frameBuf[ATTI_FRAME_RL];
        }
        /* adjust attitude value by multiply parameter */
        fpga_reg.angle_x = (short)(fpga_reg.angle_x * ATTI_VALUE_MULTIPLY);
        fpga_reg.angle_y = (short)(fpga_reg.angle_y * ATTI_VALUE_MULTIPLY);
        fpga_reg.angle_z = (short)(fpga_reg.angle_z * ATTI_VALUE_MULTIPLY);
    } 
#if 0
    else {
        for (l_cnt=0; l_cnt<l_read_cnt; l_cnt++) {
            TRACE("%02X ", responseBuf[l_cnt]);
        }
        TRACE("\n");
    }
#endif
}

/*
 * @name    setSysTime
 * @brief   setting system time with BCD data flows.
 */
static void setSysTime(unsigned char *buf)
{
    time_t time_cur,time_set;
    struct tm tm_set;

    time(&time_cur);
    TRACE("current time : %d\n", time_cur);
    convertToBrokenDownTime(&tm_set, buf);
    time_set = mktime(&tm_set);
    stime(&time_set);
    TRACE("set system time to : %d\n", time_set);
}
/*
 * @name    periodic_func
 * @brief   setting the period of periodic functions.
 */
static void periodic_func_setting(void)
{
    struct itimerval new_value,old_value;
    
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_usec = PERIODIC_FUNC_PERIOD_USEC;
    new_value.it_value.tv_sec = 0;
    new_value.it_value.tv_usec = PERIODIC_FUNC_PERIOD_USEC;
    setitimer(ITIMER_VIRTUAL, &new_value, &old_value);
}
/*
 * @name    periodic_func
 * @brief   functions that are called periodically. The period is set by 
 *          periodic_func_setting().
 */
static __sighandler_t periodic_func(void)
{
    struct timeval tv;
    struct timezone tz;
    time_t time_cur;
    unsigned char l_cycle_cnt = 0;

    /* addup register value, just for test */
    /* fpga_reg.sec += 1;
    fpga_reg.usec += 1;
    fpga_reg.deepth += 1;
    fpga_reg.angle_x += 1;
    fpga_reg.angle_y += 1;
    fpga_reg.angle_z += 1;
    */
    /* acquire deepth and attitude information from UART */
    if (++l_cycle_cnt>4){
        l_cycle_cnt = 0;
        getDeepthFromUart();
    }
    getBroadcastAttitudeFromUart();
    /* get system time */
    gettimeofday(&tv, NULL);
    /* time(&time_cur); */
    fpga_reg.sec = tv.tv_sec + CST_DIFF_IN_SEC;
    fpga_reg.usec = tv.tv_usec;
    /* write system time to fpga */
    writeSysTimeToSpi(&fpga_reg);
    /* write device deepth to fpga */
    writeDeepthToSpi(&fpga_reg);
    /* write attitude angle to fpga */
    writeAttitudeToSpi(&fpga_reg);
}
/*
 * @name all_pthread_start()
 * @brief  create all work pthread, and then start pthread.
 */
void all_pthread_start(void)
{
    /* create uart communication pthread */
    pthread_create(&pth_comm, NULL, uart_comm, NULL);
    /* join pthread to process and wait for terminate */
    TRACE("pth_common start work!\n");
    TRACE("g_recording_flag_prev is: %02X\n", g_recording_flag_prev);
    while (1) {
        if(g_recording_flag&&(g_recording_flag_prev==0)){
            g_recording_flag_prev = g_recording_flag;
            TRACE("g_recording_flag change to %d\n", g_recording_flag);
            TRACE("****************************************\n");
            TRACE("start a new recording process!\n");
            /* start recording */
            genNewFileName(storage_file_name);
            writeHeaderToFile(storage_file_name); 
            /* semaphore init */
            sem_init(&sem_notify, 0, 0);
            sem_init(&sem_complete, 0, 1);
            /* buffer parameter init */            
            buf.g_RdId = 0;
            buf.g_WtId = 0;
            buf.size = 0;
            /* start pthreads */
            pthread_create(&pth_read, NULL, data_read, NULL);
            pthread_create(&pth_write, NULL, data_write, NULL);
            pthread_create(&pth_detect, NULL, signal_detect, NULL);
        }else if(g_recording_flag_prev&&(g_recording_flag==0)){
            pthread_cancel(pth_detect);
            pthread_cancel(pth_read);
            pthread_cancel(pth_write);
            /* read data once to clear FIFO in the device */
            usleep(999999);
            read(fd_read, buf.dataBuf, READ_BUF_SIZE<<2);
            /* close signal_detect,data_read,data_write file */
            close(fd_detect);
            close(fd_read);
            close(fd_write);
            TRACE("endup a recording process!\n");
            TRACE("########################################\n");
            g_recording_flag_prev = g_recording_flag;
            TRACE("g_recording_flag change to %d\n", g_recording_flag);
        }
    }
}
/*
 * @name    all_signal_function_setting
 * @brief   setting the function call by SIGNAL
 */
void all_signal_function_setting(void)
{
    /* interrupt from keyboard */
    //signal(SIGINT, program_exit);
    /* interrupt from virtual timer */
    signal(SIGVTALRM, periodic_func);
}
/*
 * @name    system_init
 * @brief   system variables initial
 */
void system_init(void)
{
    /* variables and signal initial */
    buf.g_WtId = 0;
    buf.g_RdId = 0;
    buf.size = 0;
    memset(buf.dataBuf, 0, READ_BUF_SIZE*BLOCK_NUM);
    // mutex init
    pthread_mutex_init(&mutex, NULL);
    /* semaphore init */
    sem_init(&sem_notify, 0, 0);
    sem_init(&sem_complete, 0, 1);
}
/*****************************************************************************/
/*****************************************************************************/
/*
 * @name    main
 * @brief   main function entry
 */
int main(int argc, char* argv[])
{
    system_init();

    all_signal_function_setting();

    periodic_func_setting();

    all_pthread_start();
    
    return 0;
}

/*****************************end of file***********************************/
