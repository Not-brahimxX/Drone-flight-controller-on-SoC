#
# This file was created by Xilinx Vitis tool.
#


#
# tools
#

RM = rm -f
RMDIR = rm -rf

#
# variables
#

SDCARD_GEN = sdcard_gen
GENERATE_BIF = ::scw::generate_bif

XPFM_PATH = C:/Users/PC-CLICK-PLUS/OneDrive/Documents/ME/ELN/seminaire/Vitis_ws/ws9/test9/export/test9/test9.xpfm
SYS_CONFIG = test9
DOMAINS = standalone_domain
BIF_PATH = C:/Users/PC-CLICK-PLUS/OneDrive/Documents/ME/ELN/seminaire/Vitis_ws/ws9/test_uart_ibus_system/Debug/system.bif

BITSTREAM_PATH = C:/Users/PC-CLICK-PLUS/OneDrive/Documents/ME/ELN/seminaire/Vitis_ws/ws9/test_uart_ibus/_ide/bitstream/design_1_wrapper.bit
SD_FILES = --elf C:/Users/PC-CLICK-PLUS/OneDrive/Documents/ME/ELN/seminaire/Vitis_ws/ws9/test_uart_ibus/Debug/test_uart_ibus.elf,ps7_cortexa9_0

#
# primary build targets
#

all: sd_card

sd_card:
	-@echo Generating bif file for the system project
	generate_system_bif.bat 55315 ${XPFM_PATH} ${DOMAINS} ${BIF_PATH}
	${SDCARD_GEN} --xpfm ${XPFM_PATH} --sys_config ${SYS_CONFIG} --bif ${BIF_PATH} --bitstream ${BITSTREAM_PATH} ${SD_FILES}

clean:
	$(RMDIR) ./_sds
	${RMDIR} sd_card
	${RM} system.bif

.PHONY: all sd_card clean


