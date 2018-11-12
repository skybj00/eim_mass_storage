/*
 * @name: data_ready_module.c
 * @brief: This program is aimed at building a misc device. By reading this
 *         misc device node file /dev/data_ready, User in the userspace could
 *         get gpio value  with just a little delay. 
 * @author: william bai
 * @Date: 2018.7.29
 **/
  
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <mach/iomux-v3.h>
#include <mach/iomux-mx6dl.h>
  
#define DEV_NAME            "data_ready"
#define DATA_READY_PIN      IMX_GPIO_NR(5, 0)
#define DATA_READY_INT      gpio_to_irq(DATA_READY_PIN)
#define MODULE_NAME         "DATA_READY"
  
static int wait_flag = 0;
static unsigned long g_int_cnt = 0;

ssize_t 
data_ready_read(struct file *filp, 
                char __user *buf, 
                size_t count, 
                loff_t *ppos)
{
    static int last_flag = 0;
    static int last_cnt = 0;
    wait_flag = gpio_get_value(DATA_READY_PIN);
    if (last_flag == wait_flag) {
        last_cnt++;
    } else {
        printk("%s: gpio value : %d, previous state last %d times\n", 
                MODULE_NAME, wait_flag, last_cnt);
        last_flag = wait_flag;
        last_cnt = 0;
    }
    /* printk("%s\n", MODULE_NAME); */
    if (copy_to_user(buf, &wait_flag, 1))
        return -EFAULT;
 
    return 1;
}
 
static struct file_operations data_ready_ops = {
    .owner = THIS_MODULE,
    .read = data_ready_read,
};
 
static struct miscdevice miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEV_NAME,
    .fops = &data_ready_ops,
    .nodename = DEV_NAME,
};
  
static int 
data_ready_init(void)
{
    int ret;

    printk("%s: irq = %u\n", MODULE_NAME, DATA_READY_INT);
    /* set gpio iomux and direction */
    mxc_iomux_v3_setup_pad(MX6DL_PAD_EIM_WAIT__GPIO_5_0);
    gpio_request(DATA_READY_PIN, "data_ready_int");
    gpio_direction_input(DATA_READY_PIN);
    printk("%s: Get gpio val = %u\n", 
            MODULE_NAME, 
            gpio_get_value_cansleep(DATA_READY_PIN));
    gpio_free(DATA_READY_PIN);
    /* register misc device to kernel */
    ret = misc_register(&miscdev);
    if (ret) return ret;
                                     
    return 0;
}
 
static void 
data_ready_exit(void)
{
    misc_deregister(&miscdev);
}
 
module_init(data_ready_init);
module_exit(data_ready_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("william");
MODULE_DESCRIPTION("data ready detect");

