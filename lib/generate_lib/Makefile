
CROSS = arm-none-eabi-

CC = ${CROSS}gcc
LD = ${CROSS}ld
AR = ${CROSS}ar
STRIP =${CROSS}strip
RANLIB = ${CROSS}ranlib
MAKE = make


#export CONFIG_MAKE_PHY6222_SDK_LIB = 1
ROOT = ../..

include generate_lib.cflags
#rflib
CSRCS_RF = $(ROOT)/lib/generate_lib/source/patch.c
CSRCS_RF += $(ROOT)/components/ble_src/controller/rf_phy_driver.c

#sec boot
CSRCS_SEC = $(ROOT)/lib/generate_sec_lib/source/aes.c
CSRCS_SEC += $(ROOT)/lib/generate_sec_lib/source/phy_sec_ext.c

#efuse lib
CSRCS_SEC += $(ROOT)/lib/generate_efuse_lib/source/lib_efuse3.c


#hostlib
CSRCS_HOST = $(ROOT)/components/ble_src/host/att_client.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/att_server.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/att_util.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_centdevmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_centlinkmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_configmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_devmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_linkmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_peridevmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_perilinkmgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_simpletask.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gap_task.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gatt_client.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gatt_server.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gatt_task.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/gatt_uuid.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/l2cap_if.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/l2cap_task.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/l2cap_util.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/linkdb.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/sm_intpairing.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/sm_mgr.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/sm_pairing.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/sm_rsppairing.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/sm_task.c
CSRCS_HOST += $(ROOT)/components/ble_src/host/smp.c


CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb
CFLAGS += -ffunction-sections
#CFLAGS += --specs=rdimon.specs

#CONFIG_MAKE_PHY6222_SDK_LIB_ADVSCAN=3
#CONFIG_MAKE_PHY6222_SDK_LIB_RF=2
#CONFIG_MAKE_PHY6222_SDK_LIB_ALL=1


SUBDIRS =  
MODULE =phy6222_

#ifeq ($(CONFIG_MAKE_PHY6222_SDK_LIB), $(CONFIG_MAKE_PHY6222_SDK_LIB_ADVSCAN))
#TAR_RF	= $(ROOT)/lib/lib${MODULE}rf_advscan.a
#else
TAR_RF	= $(ROOT)/lib/lib${MODULE}rf.a
#endif
TAR_SEC	= $(ROOT)/lib/lib${MODULE}sec_boot.a
TAR_HOST = $(ROOT)/lib/lib${MODULE}host.a
#TAR	= lib${MODULE}.so

#ifeq ($(CONFIG_MAKE_PHY6222_SDK_LIB), $(CONFIG_MAKE_PHY6222_SDK_LIB_ALL))
CONFIG_MAKE_PHY6222_SDK_TEST = 1
#else
#CONFIG_MAKE_PHY6222_SDK_TEST = 2
#endif

CSRCS_RF_NODIR=$(notdir $(CSRCS_RF))
CSRCS_SEC_NODIR=$(notdir $(CSRCS_SEC))
CSRCS_HOST_NODIR=$(notdir $(CSRCS_HOST))

OBJ_RF=$(patsubst %.c,%.o,$(CSRCS_RF_NODIR))
OBJ_SEC=$(patsubst %.c,%.o,$(CSRCS_SEC_NODIR))
OBJ_HOST=$(patsubst %.c,%.o,$(CSRCS_HOST_NODIR))

all:
	@echo $(TAR_RF)
	@echo $(CSRCS_RF_NODIR)
	@echo $(OBJ_RF)

	@echo $(CSRCS_SEC_NODIR)
	@echo $(OBJ_SEC)

	@echo $(CSRCS_HOST_NODIR)
	@echo $(OBJ_HOST)
	
	${CC} -c -Os -g3 $(CSRCS_RF) $(CFLAGS) $(DEF) $(INC)
	${CC} -c -Os -g3 $(CSRCS_SEC) $(CFLAGS) $(DEF) $(INC)
	${AR} r ${TAR_RF} $(OBJ_RF)
	${AR} r ${TAR_SEC} $(OBJ_SEC)
#ifeq ($(CONFIG_MAKE_PHY6222_SDK_LIB), $(CONFIG_MAKE_PHY6222_SDK_LIB_ALL))
	${CC} -c -Os -g3 $(CSRCS_HOST) $(CFLAGS) $(DEF) $(INC)
	${AR} r ${TAR_HOST} $(OBJ_HOST)
#endif
clean:
	-rm *.o
	-rm $(ROOT)/lib/*.a

install:


