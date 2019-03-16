-include ../Make.inc

TARGET := gpio_spi_20180930
obj-m := $(TARGET).o
$(TARGET)-objs = gpio_spi_module.o 
default:
	make $(EXTRA_CFLAGS) -C $(SRC_DIR) M=$(PWD) modules

install:
	insmod $(OBJS)
uninstall:
	rmmod $(OBJS)
clean:
	rm -rf *.o *.o.cmd *.ko *.mod.c *.order *.symvers
	rm -rf .tmp_versions .gpio*
