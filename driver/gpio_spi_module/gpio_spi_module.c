#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/string.h>  /* memcpy */
#include <linux/ioport.h> /* request_mem_region() */
/* header for gpio mux */
#include <mach/gpio.h>
#include <mach/iomux-v3.h>
#include <mach/iomux-mx6q.h>

/* GPIO PIN DEFINE */
#define SCLK_PIN    IMX_GPIO_NR(3, 0)
#define MISO_PIN    IMX_GPIO_NR(3, 1)
#define MOSI_PIN    IMX_GPIO_NR(3, 2)
#define CS_PIN      IMX_GPIO_NR(3, 3)
/* GPIO OPERATION DEFINE */
#define SCLK_HIGH   (gpio_set_value(SCLK_PIN, 1))
#define SCLK_LOW    (gpio_set_value(SCLK_PIN, 0))
#define MOSI_HIGH   (gpio_set_value(MOSI_PIN, 1))
#define MOSI_LOW    (gpio_set_value(MOSI_PIN, 0))
#define MISO_VALUE   (gpio_get_value(MISO_PIN))
#define CS_HIGH     (gpio_set_value(CS_PIN, 1))
#define CS_LOW      (gpio_set_value(CS_PIN, 0))

#define DEBUG_PRINT 1
#if DEBUG_PRINT
#define    TRACE(fmt,args...) printk(KERN_DEBUG fmt,##args)
#else
#define    TRACE(fmt,args...) 
#endif
#define GPIO_SPI_LEN (0x100)
#define GPIO_SPI_MINOR (0)
#define MODULE_NAME "gpio_spi"

static struct class *gpio_spi_class;
static struct device *gpio_spi_dev;
int major;
unsigned int reg_down;
unsigned int reg_up;

/*****************************Private function definition*****************/
static inline void mosi_output(unsigned int reg)
{
    if(reg&0x80000000){ 
        MOSI_HIGH;
        /* TRACE("%s: 1\n", MODULE_NAME); */
    }else{ 
        MOSI_LOW;
        /* TRACE("%s: 0\n", MODULE_NAME); */
    }
}
static inline void sclk_delay(void)
{
    int l_cnt = 1;
    while(l_cnt--);
}
/*
 * @name    gpio_spi_transmit
 * @brief   SPI transmit via 3 GPIO pins, CPOL=1,CPHA=1, band width 32bits
 */
static int gpio_spi_transmit(unsigned int reg_in, unsigned int *reg_out)
{
    int l_bit_cnt = 0;

    /* pull cs pin low */
    CS_LOW;
    sclk_delay();
    /* clear reg_out */
    *reg_out = 0;
    /* transmit 32 bit to device */
    for (l_bit_cnt=0; l_bit_cnt<64; l_bit_cnt++) {
        if (l_bit_cnt%2==0) {
            /* falling edge of sclk */
            if (1) {
                mosi_output(reg_in);
                reg_in <<= 1;
            }
            SCLK_LOW;
        } else {
            /* rising edge of sclk */
            if (l_bit_cnt > 16) {
                *reg_out <<= 1;
                *reg_out |= MISO_VALUE;
            }
            SCLK_HIGH;
        }
        sclk_delay();
    }
    /* pull cs pin high */
    CS_HIGH;
    sclk_delay();

    return 0;
}
static int 
gpio_spi_open(struct inode *inode, struct file *file)
{
    /* TRACE("%s: open device succeed!\n", MODULE_NAME); */
    return 0;
}

static ssize_t 
gpio_spi_write(struct file *file, 
            const char __user *buf, 
            size_t count, loff_t * ppos)
{		
    /* copy data from user-space with little-endian */
    copy_from_user((unsigned char*)(&reg_down), buf, 4);
    gpio_spi_transmit(reg_down, &reg_up);
    if(reg_down>>24==0x80||reg_down>>24==0x81){
        TRACE("%s: bytes down: 0x%08X, bytes up : 0x%08X\n", 
            MODULE_NAME, reg_down, reg_up);
    }
    return 0;
}
static ssize_t 
gpio_spi_read(struct file *file, 
            const char __user *buf, 
            size_t count, loff_t *ppos)
{
    copy_to_user(buf, (unsigned char*)(&reg_up), 4);
    TRACE("%s: read value from device, bytes up : 0x%08X\n", 
        MODULE_NAME, reg_up);

    return 0;
}
static struct file_operations gpio_spi_fops = {
        .owner  =   THIS_MODULE,   
        .open   =   gpio_spi_open,       
	.write  =   gpio_spi_write,
	.read   =   gpio_spi_read
};
static int gpio_spi_init(void)
{
    major = register_chrdev(0, MODULE_NAME, &gpio_spi_fops);
    gpio_spi_class = class_create(THIS_MODULE, MODULE_NAME);
    gpio_spi_dev = device_create(gpio_spi_class, 
                                NULL, 
                                MKDEV(major, GPIO_SPI_MINOR), 
                                NULL, 
                                MODULE_NAME);
    /* GPIO REQUEST */
    gpio_request(SCLK_PIN, "sysfs");
    gpio_request(MOSI_PIN, "sysfs");
    gpio_request(MISO_PIN, "sysfs");
    gpio_request(CS_PIN, "sysfs");
    /* initial GPIO pins SCLK,MISO,MOSI for spi */
    mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_DA0__GPIO_3_0);
    mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_DA1__GPIO_3_1);
    mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_DA2__GPIO_3_2);
    mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_DA3__GPIO_3_3);
    gpio_direction_input(MISO_PIN);
    gpio_direction_output(SCLK_PIN, 1); /* output HIGH */
    gpio_direction_output(MOSI_PIN, 1); /* output HIGH */
    gpio_direction_output(CS_PIN, 1);
    /* initial SCLK,MOSI value */
    gpio_set_value(SCLK_PIN, 1);
    gpio_set_value(MOSI_PIN, 1);
    gpio_set_value(CS_PIN, 1);
    
    return 0;
}

static int gpio_spi_exit(void)
{
    unregister_chrdev(major, MODULE_NAME); 
    device_unregister(gpio_spi_dev);
    class_destroy(gpio_spi_class);
    gpio_free(SCLK_PIN);
    gpio_free(MOSI_PIN);
    gpio_free(MISO_PIN);
    gpio_free(CS_PIN);
    TRACE("%s: module driver removed finished!\n", MODULE_NAME);
    return 0;
}

module_init(gpio_spi_init);
module_exit(gpio_spi_exit);
MODULE_LICENSE("GPL");	
MODULE_AUTHOR("William Bai");





