include ../Make.inc

TARGET := iomuxc_cfg_20180901
obj-m := $(TARGET).o
$(TARGET)-objs = iomuxc_module.o 
default:
	make $(EXTRA_CFLAGS) -C $(SRC_DIR) M=$(PWD) modules

install:
	insmod $(OBJS)
uninstall:
	rmmod $(OBJS)
clean:
	rm -rf *.o *.ko *.mod.c *.order *.symvers *o.cmd
	rm -rf .tmp_versions .iomux*
