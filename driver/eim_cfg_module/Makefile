include ../Make.inc

TARGET := eim_cfg_20180901
obj-m := $(TARGET).o
$(TARGET)-objs = eim_cfg_module.o 
default:
	make $(EXTRA_CFLAGS) -C $(SRC_DIR) M=$(PWD) modules

install:
	insmod $(OBJS)
uninstall:
	rmmod $(OBJS)
clean:
	rm -rf *.o *.o.cmd *.ko *.mod.c *.order *.symvers
	rm -rf .tmp_versions .eim*
