#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/string.h>  /* memcpy */
#include <linux/ioport.h> /* request_mem_region() */


#define HW_EIM_CS_BASE (0x020e0000)
#define EIM_CS_LEN (0x1000)
#define EIM_CS_MINOR (0)
#define MODULE_NAME "iomux_cfg"

volatile unsigned long *eim_cs_remap_addr=NULL;

static struct class *eim_cs_class;
static struct device *eim_cs_dev;
int major;
unsigned int reg_val = 0;

static int eim_cs_open(struct inode *inode, struct file *file)
{
	file->private_data = (void*)eim_cs_remap_addr;
	printk("%s: open device succeed!\n", MODULE_NAME);
       	return 0;
}

static ssize_t eim_cs_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos)
{		
	uint32_t *addr_base = (uint32_t*)(file->private_data)+(*ppos);

	copy_from_user((unsigned char*)(&reg_val), buf, 4);
	writel(reg_val, addr_base);
	printk("%s: write %d byte(s) to device 0x%08X, value : %08Xh\n", 
	    MODULE_NAME, count, addr_base, reg_val);

	return 0;
}
static ssize_t eim_cs_read(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	uint32_t *addr_base = (uint32_t*)(file->private_data)+(*ppos);

	reg_val = readl(addr_base);
        copy_to_user(buf, (unsigned char*)(&reg_val), 4);
	printk("%s: read %d byte(s) from device 0x%08X, value : %08Xh\n", 
	    MODULE_NAME, count, addr_base, reg_val);

	return 0;
}
static loff_t eim_cs_llseek(struct file *filp, loff_t offset, int orig)
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
static struct file_operations eim_cs_fops = {
        .owner  =   THIS_MODULE,   
        .open   =   eim_cs_open,       
	.write  =   eim_cs_write,
	.read   =   eim_cs_read,
        .llseek =   eim_cs_llseek,	   
};
static int eim_cs_init(void)
{
	major=register_chrdev(0, MODULE_NAME, &eim_cs_fops);
        eim_cs_class = class_create(THIS_MODULE, MODULE_NAME);
        eim_cs_dev = device_create(eim_cs_class, NULL, MKDEV(major, EIM_CS_MINOR), NULL, MODULE_NAME);
	if (!request_mem_region(HW_EIM_CS_BASE, EIM_CS_LEN, MODULE_NAME)) {
	    printk("%s: request io memory failed!\n", MODULE_NAME);
	} else {
	    printk("%s: request io memory succeed! start: 0x%08X, len: 0x%X\n", 
		MODULE_NAME, HW_EIM_CS_BASE, EIM_CS_LEN);
	}
	eim_cs_remap_addr = (volatile unsigned long *)ioremap(HW_EIM_CS_BASE, EIM_CS_LEN);
	printk("%s: phy to virt addr: 0x%X --> 0x%X\n", MODULE_NAME, HW_EIM_CS_BASE, eim_cs_remap_addr);

        return 0;
}

static int eim_cs_exit(void)
{
	unregister_chrdev(major, MODULE_NAME); 
        device_unregister(eim_cs_dev);
        class_destroy(eim_cs_class);

	release_mem_region(HW_EIM_CS_BASE, EIM_CS_LEN);
        iounmap(eim_cs_remap_addr);

	printk("%s: module driver removed finished!\n", MODULE_NAME);
	return 0;
}

module_init(eim_cs_init);
module_exit(eim_cs_exit);
MODULE_LICENSE("GPL");	





