/*
 * This program contain three part of register initialization: 
 * CLK register init, IOMUX register init, EIM register init.
 * each part is achieved in a function, which would be called by
 * main().
 *
 * author: william white
 * date :10/29/18
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "reg_eim.h"

#define EIM_CLK_44M
#define WRITE   1
#define READ    0

static void setGpioToEimPin(unsigned int offset, unsigned int value, unsigned char flag);
static void eim_init(void);
static void iomuxc_init(void);
static void ccm_init(void);

/*
 * main(), program entry
 */
int main(void)
{
    volatile unsigned int i;

    ccm_init();
    sleep(2);
    eim_init();
    sleep(2);
    iomuxc_init();
    sleep(2);

    return 0;
}
/*
 * This function is used to config CCM_CCGR6 registers, which at physical address
 * 0x020C_4080. By setting bit[11-10], the clock of eim module is enabled.
 * To finish this instrction, write 0x0000_0C03 to CCM_CCGR6. 
 * To run this program, ccm kernel module must be load to kernel. This kernel module
 * remapped IO memory (0x020C_4000~0x020C_4100) to virtual memory region, and create
 * a device node named "/dev/ccm_cfg".
 */
static void ccm_init(void)
{
    unsigned int value = 0;
    unsigned int res = 0;
    int fd_ccm = 0;

    char file_name[20] = "/dev/ccm_cfg";

    fd_ccm = open(file_name, O_RDWR);
    if (fd_ccm < 0) {
        //printf("open device %s failed!\n", file_name);
    }

/* set bit[11] and bit[10] to enable eim clock */
    value = 0x00000c03;
    lseek(fd_ccm, 0x80, SEEK_SET);
    write(fd_ccm, (unsigned char*)(&value), 4);

    lseek(fd_ccm, 0x80, SEEK_SET);
    read(fd_ccm, (unsigned char*)(&res), 4);
    //printf("CCM_CCGR6: 0x%08X\n", res);

    close(fd_ccm);
}

/*
 * This function configure EIM working mode by changing eim registers.
 * EIM is set to synchorous burst read/write, data applied at DATA[15..0],
 * eim burst clock is divided by 1,2,3,4, and eim source clk frequency is 
 * 132MHz. Eim address line is not used, because burst mode just generate 
 * base address for every 8 word.
 */ 
static void eim_init(void)
{
    unsigned int reg[6] = {0};
    int fd = 0;
    int ret;

    fd = open("/dev/eim_cfg", O_RDWR);
    if (fd < 0) {
        printf("open file failed!\n");
    }

#if 1
/* eim config for Sync burst Read Mode */
    reg[0] = GCR1_PSZ_128 | GCR1_WP_ALLOW | GCR1_GBC_2 | GCR1_AUS_0
            | GCR1_CSREC_2 | GCR1_SP_0 | GCR1_DSZ_1 | GCR1_BCS_0
            | GCR1_BCD_DIV_1 | GCR1_WC_0 |  GCR1_BL_8WORDS
            | GCR1_CREP_LOW | GCR1_CRE_DIS | GCR1_RFL_1 | GCR1_WFL_1
            | GCR1_MUM_DIS | GCR1_SRD_SYNC | GCR1_SWR_SYNC | GCR1_CSEN_EN;

    reg[1] = GCR2_MUX_BYP_DIS | GCR2_DAP_ACTIVE_HIGH | GCR2_DAE_DIS
            | GCR2_DAPS_0 | GCR2_ADH_0;

    reg[2] = RCR1_RWSC_0 | RCR1_RAL_DIS | RCR1_RADVN_0
            | RCR1_OEA_0 | RCR1_OEN_0 | RCR1_RCSA_0 | RCR1_RCSN_0;
    reg[3] = RCR2_APR_DIS | RCR2_PAT_0 | RCR2_RL_0
            | RCR2_RBEA_0 | RCR2_RBE_DIS | RCR2_RBEN_0;

    reg[4] = 0x00000000;
    reg[5] = 0x00000000;

#ifdef EIM_CLK_133M
    reg[0] |= GCR1_BCD_DIV_1 | GCR1_BCS_0;
    reg[2] |= RCR1_RWSC_5 | RCR1_OEA_2;
#endif
#ifdef EIM_CLK_66M
    reg[0] |= GCR1_BCD_DIV_2 | GCR1_BCS_0;
    reg[2] |= RCR1_RWSC_3 | RCR1_OEA_3;
#endif
#ifdef EIM_CLK_44M
    reg[0] |= GCR1_BCD_DIV_3 | GCR1_BCS_0;
    reg[2] |= RCR1_RWSC_3 | RCR1_OEA_4;
#endif
    lseek(fd, EIM_CS0GCR1, SEEK_SET);
    write(fd, (unsigned char*)(&reg[0]), 4);
    lseek(fd, EIM_CS0GCR2, SEEK_SET);
    write(fd, (unsigned char*)(&reg[1]), 4);
    lseek(fd, EIM_CS0RCR1, SEEK_SET);
    write(fd, (unsigned char*)(&reg[2]), 4);
    lseek(fd, EIM_CS0RCR2, SEEK_SET);
    write(fd, (unsigned char*)(&reg[3]), 4);
    lseek(fd, EIM_CS0WCR1, SEEK_SET);
    write(fd, (unsigned char*)(&reg[4]), 4);
    lseek(fd, EIM_CS0WCR2, SEEK_SET);
    write(fd, (unsigned char*)(&reg[5]), 4);
#endif

    ret = close(fd);
    if (ret < 0) {                                                                                  printf("error close the device! err: %d\n", ret);
    }
}

static void setGpioToEimPin(unsigned int offset, unsigned int value, unsigned char flag)
{   
    unsigned int res = 0;
    int fd = 0;
                
    char file_name[20] = "/dev/iomux_cfg";
        
    fd = open(file_name, O_RDWR);
    if (fd < 0) {
        printf("open device %s failed!\n", file_name);
        return;
    }

    if(flag){
        lseek(fd, offset, SEEK_SET);
        write(fd, (unsigned char*)(&value), 4);
    }
    lseek(fd, offset, SEEK_SET);
    read(fd, (unsigned char*)(&res), 4);
    //printf("iomuxc offset %02XH : %08X\n", offset, res);
    close(fd);
}

static void iomuxc_init(void)
{
    unsigned int value = 0;
    unsigned int res = 0;
    int fd = 0;
    unsigned char l_flag = 0;

    char file_name[20] = "/dev/iomux_cfg";

    fd = open(file_name, O_RDWR);
    if (fd < 0) {
        printf("open device %s failed!\n", file_name);
        return;
    }

    lseek(fd, 0x4, SEEK_SET);
    read(fd, (unsigned char*)(&res), 4);
    printf("IOMUXC_GPR1: 0x%08X\n", res);

    value = res & (~0x00000fff);
    value |= 0x01b;
    lseek(fd, 0x4, SEEK_SET);
    write(fd, (unsigned char*)(&value), 4);

    lseek(fd, 0x4, SEEK_SET);
    read(fd, (unsigned char*)(&res), 4);
    printf("IOMUXC_GPR1: 0x%08X\n", res);

    close(fd);

    l_flag = WRITE;
    setGpioToEimPin(IOMUXC_DATA_READY, 0x05, l_flag);   /* EIM_WAIT */
    setGpioToEimPin(IOMUXC_SPI_SCLK, 0x05, l_flag);   /* EIM_DA0 */
    setGpioToEimPin(IOMUXC_SPI_MISO, 0x05, l_flag);   /* EIM_DA1 */
    setGpioToEimPin(IOMUXC_SPI_MOSI, 0x05, l_flag);   /* EIM_DA2 */
    setGpioToEimPin(IOMUXC_SPI_NSS, 0x05, l_flag);   /* EIM_DA3 */
    
    setGpioToEimPin(IOMUXC_EIM_BCLK, 0x10, l_flag); 
    setGpioToEimPin(IOMUXC_EIM_OE, 0x10, l_flag);
    setGpioToEimPin(IOMUXC_EIM_CS0, 0x10, l_flag);
    setGpioToEimPin(IOMUXC_EIM_RW, 0x10, l_flag);
    
    setGpioToEimPin(IOMUXC_EIM_D0, 0x11, l_flag);   /* CSI0_DATA_EN */
    setGpioToEimPin(IOMUXC_EIM_D1, 0x11, l_flag);   /* CSI0_VSYNC */
    setGpioToEimPin(IOMUXC_EIM_D2, 0x11, l_flag);   /* CSI0_DATA4 */
    setGpioToEimPin(IOMUXC_EIM_D3, 0x11, l_flag);   /* CSI0_DATA5 */
    setGpioToEimPin(IOMUXC_EIM_D4, 0x11, l_flag);   /* CSI0_DATA6 */
    setGpioToEimPin(IOMUXC_EIM_D5, 0x11, l_flag);   /* CSI0_DATA7 */
    setGpioToEimPin(IOMUXC_EIM_D6, 0x11, l_flag);   /* CSI0_DATA8 */
    setGpioToEimPin(IOMUXC_EIM_D7, 0x11, l_flag);   /* CSI0_DATA9 */
    setGpioToEimPin(IOMUXC_EIM_D8, 0x11, l_flag);   /* CSI0_DATA12 */
    setGpioToEimPin(IOMUXC_EIM_D9, 0x11, l_flag);   /* CSI0_DATA13 */
    setGpioToEimPin(IOMUXC_EIM_D10, 0x11, l_flag);  /* CSI0_DATA14 */
    setGpioToEimPin(IOMUXC_EIM_D11, 0x11, l_flag);  /* CSI0_DATA15 */
    setGpioToEimPin(IOMUXC_EIM_D12, 0x11, l_flag);  /* CSI0_DATA16 */
    setGpioToEimPin(IOMUXC_EIM_D13, 0x11, l_flag);  /* CSI0_DATA17 */
    setGpioToEimPin(IOMUXC_EIM_D14, 0x11, l_flag);  /* CSI0_DATA18 */
    setGpioToEimPin(IOMUXC_EIM_D15, 0x11, l_flag);  /* CSI0_DATA19 */
}

