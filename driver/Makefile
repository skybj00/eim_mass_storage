
SRC=$(PWD)
CCM=${SRC}/ccm_module/
RDY=${SRC}/data_ready_module/
CFG=${SRC}/eim_cfg_module/
RW=${SRC}/eim_rw_module/
SPI=${SRC}/gpio_spi_module/
IOMUXC=${SRC}/iomuxc_module/

all:
	make ccm; make ready; make eim_cfg; make eim_rw; make gpio_spi;
	make iomuxc;
ccm:
	cd ${CCM}; make
ready:
	cd ${RDY}; make
eim_cfg:
	cd ${CFG}; make
eim_rw:
	cd ${RW}; make
gpio_spi:
	cd ${SPI}; make
iomuxc:
	cd ${IOMUXC}; make

clean_all:
	cd ${CCM}; make clean;
	cd ${RDY}; make clean;
	cd ${CFG}; make clean;
	cd ${RW}; make clean;
	cd ${SPI}; make clean;
	cd ${IOMUXC}; make clean;
