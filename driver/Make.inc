# define common configuration for linux kernel modules compile
CC = ${CROSS_COMPILE}gcc

DRIVER_ROOT = /home/ubuntu/work_imx6q/
SRC_DIR = /home/ubuntu/work_imx6q/linux-3.0.35/
TOOL_DIR = /home/ubuntu/work_imx6q/fsl-linaro-toolchain/
INCLUDE_DIR = ${TOOL_DIR}arm-fsl-linux-gnueabi/multi-libs/usr

EXTRA_CFLAGS = -Wall -I$(INCLUDE_DIR) 
