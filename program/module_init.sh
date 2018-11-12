#!/bin/bash

############################################################################
# if module loading is finished, write string "yes" to file, which would tell 
# another program to start main prgram.

SRC=/root/eim_storage/

MODULE_DIR=${SRC}kernel_module/
BINARY_DIR=${SRC}bin/
STARTUP_DIR=${SRC}startup/
MODULE_CCM=ccm_20180901.ko
MODULE_EIM_CFG=eim_cfg_20180901.ko
MODULE_EIM_RW=eim_rw_20180901.ko
MODULE_IOMUXC=iomuxc_cfg_20180901.ko
MODULE_DETECT=data_ready_20180930.ko
MODULE_SPI=gpio_spi_20180930.ko
MAIN_PROGRAM=${BINARY_DIR}./eim_data_storage_v3

function PidFind()
{
    PIDCOUNT=`ps -ef | grep $1 | grep -v "grep" | grep -v $0 | awk '{print $2}' | wc -l`;
    if [ ${PIDCOUNT} -gt 1 ] ; then
        echo "There are too many process contains name[$1]"  
    elif [ ${PIDCOUNT} -le 0 ] ; then
        echo "No such process[$1]!"  
    else
        PID=`ps -ef | grep $1 | grep -v "grep" | grep -v ".sh" | awk '{print $2}'` ;
        echo "Find the PID of this progress!--- process:$1 PID=[${PID}] ";
        echo "Kill the process $1 ...";
        kill -9  ${PID};
        echo "kill -9 ${PID} $1 done!";
    fi
}

if [ "$1" = "clean" ]; then
    rmmod $MODULE_CCM
    rmmod $MODULE_EIM_CFG
    rmmod $MODULE_EIM_RW
    rmmod $MODULE_IOMUXC
    rmmod $MODULE_DETECT
    rmmod $MODULE_SPI
else
    MODULE_CNT=$(lsmod|wc -l)
    if [ $MODULE_CNT -lt 2 ]; then
        insmod ${MODULE_DIR}${MODULE_CCM}
        insmod ${MODULE_DIR}$MODULE_EIM_CFG
        insmod ${MODULE_DIR}$MODULE_EIM_RW
        insmod ${MODULE_DIR}$MODULE_IOMUXC
        insmod ${MODULE_DIR}$MODULE_DETECT
        insmod ${MODULE_DIR}$MODULE_SPI
        sleep 10
        ${BINARY_DIR}./regs_init
        # mount SSD to /mnt/ssd
        mount /dev/sda1 /mnt/ssd
    fi
    # if SSD mount failed! exit the program
    MOUNT_CHECK=$(df -h|grep "ssd"|wc -l)
    if [ $MOUNT_CHECK -gt 0 ]; then
        echo "SSD mount succeed!"
        # close main program
        PidFind eim
        # execute main program
        ${MAIN_PROGRAM}
    else
        echo "SSD mount failed!"
    fi
fi

exit 0
