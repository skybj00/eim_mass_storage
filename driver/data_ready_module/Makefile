-include ../Make.inc

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
	rm -rf .tmp_versions .data_ready*
