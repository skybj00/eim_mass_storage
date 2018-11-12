CC=$(CROSS_COMPILE)gcc
INCLUDE_DIR = /usr/local/bin/fsl-linux-toolchain/arm-fsl-linux-gnueabi/multi-libs/usr
#EXTRA_CFLAGS = -Wall -I$(INCLUDE_DIR)
EXTRA_CFLAGS = -Wall
SRC_DIR = /usr/src/linux-3.0.35_recorder_710
TARGET := data_ready_20180930

obj-m := $(TARGET).o
$(TARGET)-objs = data_ready_module.o 
default:
	make $(EXTRA_CFLAGS) -C $(SRC_DIR) M=$(PWD) modules

install:
	insmod $(OBJS)
uninstall:
	rmmod $(OBJS)
clean:
	rm -rf *.o *.o.cmd *.ko *.mod.c *.order *.symvers
