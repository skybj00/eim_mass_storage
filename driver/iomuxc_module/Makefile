CC=$(CROSS_COMPILE)gcc
INCLUDE_DIR = /usr/local/bin/CodeSourcery/fsl-linux-toolchain/arm-fsl-linux-gnueabi/multi-libs/usr
EXTRA_CFLAGS = -Wall -I$(INCLUDE_DIR)
SRC_DIR = /usr/src/linux-3.0.35_recorder_710
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
