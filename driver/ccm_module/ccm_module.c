#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/sh_clk.h>
#include <linux/device.h>
#include <linux/string.h>  /* memcpy */
#include <linux/ioport.h> /* request_mem_region() */


#define HW_CCM_CFG_BASE (0x020c4000)
#define CCM_CFG_LEN (0x100)
#define CCM_CFG_MINOR (0)
#define MODULE_NAME "ccm_cfg"

volatile unsigned long *  led_reg=NULL;
volatile unsigned long *ccm_cfg_remap_addr=NULL;

static struct class *ccm_cfg_class;
static struct device *ccm_cfg_dev;
int major;
unsigned int reg_val = 0;

static int ccm_cfg_open(struct inode *inode, struct file *file)
{
	file->private_data = (void*)ccm_cfg_remap_addr;
	printk("%s: open device succeed!\n", MODULE_NAME);
       	return 0;
}

static ssize_t ccm_cfg_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos)
{		
	uint32_t *addr_base = (uint32_t*)(file->private_data)+(*ppos);

	copy_from_user((unsigned char*)(&reg_val), buf, 4);
	writel(reg_val, addr_base);
	printk("%s: write %d byte(s) to device 0x%08X, value : %08Xh\n", 
	    MODULE_NAME, count, addr_base, reg_val);

	return 0;
}
static ssize_t ccm_cfg_read(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	uint32_t *addr_base = (uint32_t*)(file->private_data)+(*ppos);

	reg_val = readl(addr_base);
        copy_to_user(buf, (unsigned char*)(&reg_val), 4);
	printk("%s: read %d byte(s) from device 0x%08X, value : %08Xh\n", 
	    MODULE_NAME, count, addr_base, reg_val);

	return 0;
}
static loff_t ccm_cfg_llseek(struct file *filp, loff_t offset, int orig)
{
    loff_t ret = 0;
    switch (orig) {
        case 0: {
            if (offset < 0) {
                ret = -EINVAL;
                break;
            }
            filp->f_pos = (unsigned int)offset/4;
            ret = filp->f_pos;
            break;
        }
        case 1: {
            filp->f_pos += (unsigned int)offset/4;
            ret = filp->f_pos;
            break;
        }
        default: {
            ret = -EINVAL;
            break;
        }
    }
    return ret;
}
static struct file_operations ccm_cfg_fops = {
        .owner  =   THIS_MODULE,   
        .open   =   ccm_cfg_open,       
	.write  =   ccm_cfg_write,
	.read   =   ccm_cfg_read,
        .llseek =   ccm_cfg_llseek,	   
};
static int ccm_cfg_init(void)
{
	unsigned int reg;
	struct clk *clk;
	unsigned int rate;

	major=register_chrdev(0, MODULE_NAME, &ccm_cfg_fops);
        ccm_cfg_class = class_create(THIS_MODULE, MODULE_NAME);
        ccm_cfg_dev = device_create(ccm_cfg_class, NULL, MKDEV(major, CCM_CFG_MINOR), NULL, MODULE_NAME);
	if (!request_mem_region(HW_CCM_CFG_BASE, CCM_CFG_LEN, MODULE_NAME)) {
	    printk("%s: request io memory failed!\n", MODULE_NAME);
	} else {
	    printk("%s: request io memory succeed! start: 0x%08X, len: 0x%X\n", 
		MODULE_NAME, HW_CCM_CFG_BASE, CCM_CFG_LEN);
	}
	ccm_cfg_remap_addr = (volatile unsigned long *)ioremap(HW_CCM_CFG_BASE, CCM_CFG_LEN);
	printk("%s: phy to virt addr: 0x%X --> 0x%X\n", MODULE_NAME, HW_CCM_CFG_BASE, ccm_cfg_remap_addr);

        // set CCM_CCGR6, offset 0x80
	reg = readl(ccm_cfg_remap_addr + 0x80);
	reg |= 0x00000c00;
	writel(reg, ccm_cfg_remap_addr + 0x80);

	reg = readl(ccm_cfg_remap_addr + 0x1c);
	printk("%s: CCM_CSCMR1 value : 0x%08X\n", MODULE_NAME, reg);

	clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(clk)) {
	    printk("%s: eim_slow_clk not found\n", MODULE_NAME);
	}
	rate = clk_get_rate(clk);
	if (rate != 132000000) {
	    printk("%s: eim_slow_clk not set to 132MHz!", MODULE_NAME);
	}
 
        return 0;
}

static int ccm_cfg_exit(void)
{
	unregister_chrdev(major, MODULE_NAME); 
        device_unregister(ccm_cfg_dev);
        class_destroy(ccm_cfg_class);

	release_mem_region(HW_CCM_CFG_BASE, CCM_CFG_LEN);
        iounmap(ccm_cfg_remap_addr);

	printk("%s: module driver removed finished!\n", MODULE_NAME);
	return 0;
}

module_init(ccm_cfg_init);
module_exit(ccm_cfg_exit);
MODULE_LICENSE("GPL");	





