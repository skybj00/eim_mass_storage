-include ../Make.inc

TARGET := eim_rw_20180927
obj-m := $(TARGET).o
$(TARGET)-objs = eim_rw_module.o 
default:
	make $(EXTRA_CFLAGS) -C $(SRC_DIR) M=$(PWD) modules

install:
	insmod $(OBJS)
uninstall:
	rmmod $(OBJS)
clean:
	rm -rf *.o *.ko *.mod.c *.order *.symvers *o.cmd
	rm -rf .tmp_versions .eim*
