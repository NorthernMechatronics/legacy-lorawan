include application.mk

ifndef NM_SDK
    $(error NM_SDK location not defined)
endif

ifndef AMBIQ_SDK
    $(error AmbiqSuite SDK location not defined)
endif

ifndef CORDIO
    $(error ARM BLE Cordio Stack location not defined)
endif

ifndef UECC
    $(error Micro ECC library location not defined)
endif

ifndef FREERTOS
    $(error FreeRTOS location not defined)
endif

ifndef LORAMAC
    $(error LoRaMAC Node library location not defined)
endif

include makedefs/nm_common.mk
include makedefs/nm_application.mk
include makedefs/nm_cordio.mk
include makedefs/nm_loramac.mk

LDSCRIPT := ./ldscript.ld
LDSCRIPT_SECURE := ./ldscript_secure.ld

ifdef DEBUG
    BUILDDIR := ./debug
    BSP_LIB  := am_bsp-dev
else
    BUILDDIR := ./release
    BSP_LIB  := am_bsp
endif

BSP_C := $(BSP_DIR)/am_bsp_pins.c
BSP_H := $(BSP_DIR)/am_bsp_pins.h

INCLUDES += -I$(BSP_DIR)
INCLUDES += -I$(NM_SDK)/bsp/devices
INCLUDES += -I$(NM_SDK)/platform

INCLUDES += -I$(CORDIO_PROFILES)/sources/apps
INCLUDES += -I$(CORDIO_PROFILES)/sources/apps/app
INCLUDES += -I$(CORDIO_PROFILES)/sources/apps/app/common

INCLUDES += -I.

VPATH += $(NM_SDK)/platform
VPATH += .

SRC += startup_gcc.c
SRC += main.c
SRC += build_timestamp.c

CSRC = $(filter %.c, $(SRC))
ASRC = $(filter %.s, $(SRC))

OBJS  = $(CSRC:%.c=$(BUILDDIR)/%.o)
OBJS += $(ASRC:%.s=$(BUILDDIR)/%.o)

DEPS  = $(CSRC:%.c=$(BUILDDIR)/%.d)
DEPS += $(ASRC:%.s=$(BUILDDIR)/%.d)

CFLAGS += $(INCLUDES)
CFLAGS += $(DEFINES)

LFLAGS += -Wl,--start-group
LFLAGS += -L$(AMBIQ_SDK)/CMSIS/ARM/Lib/ARM
LFLAGS += -L$(NM_SDK)/build
LFLAGS += -L$(BSP_DIR)/$(BUILDDIR)
LFLAGS += -larm_cortexM4lf_math
LFLAGS += -lm
LFLAGS += -lc
LFLAGS += -lgcc
LFLAGS += $(LIBS)
LFLAGS += -l$(BSP_LIB)
LFLAGS += --specs=nano.specs
LFLAGS += --specs=nosys.specs
LFLAGS += -Wl,--end-group

all: directories bsp $(BUILDDIR)/$(TARGET).bin

ota: directories bsp $(BUILDDIR)/$(TARGET_OTA).bin

wire: directories bsp $(BUILDDIR)/$(TARGET_WIRE).bin

secure: directories bsp $(BUILDDIR)/$(TARGET_SECURE).bin

directories: $(BUILDDIR)

$(BUILDDIR):
	@$(MKDIR) $@

bsp: $(BSP_DIR)/$(BUILDDIR)/lib$(BSP_LIB).a

$(BSP_DIR)/$(BUILDDIR)/lib$(BSP_LIB).a:
	$(MAKE) -C $(BSP_DIR) AMBIQ_SDK=$(AMBIQ_SDK)

$(BUILDDIR)/%.o: %.c $(BUILDDIR)/%.d $(INCS) $(BSP_C) $(BSP_H)
	@echo "Compiling $(COMPILERNAME) $<"
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.s $(BUILDDIR)/%.d $(INCS) $(BSP_C) $(BSP_H)
	@echo "Assembling $(COMPILERNAME) $<"
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILDDIR)/$(TARGET).axf: $(OBJS)
	@echo "Linking $@"
	$(CC) -Wl,-T,$(LDSCRIPT) -o $@ $(OBJS) $(LFLAGS)

$(BUILDDIR)/$(TARGET_SECURE)_0xC100.axf: $(OBJS)
	@echo "Linking $@"
	$(CC) -Wl,-T,$(LDSCRIPT_SECURE) -o $@ $(OBJS) $(LFLAGS)

$(BUILDDIR)/$(TARGET).bin: $(BUILDDIR)/$(TARGET).axf
	$(OCP) $(OCPFLAGS) $< $@
	$(OD) $(ODFLAGS) $< > $(BUILDDIR)/$(TARGET).lst

$(BUILDDIR)/$(TARGET_SECURE)_0xC100.bin: $(BUILDDIR)/$(TARGET_SECURE)_0xC100.axf
	$(OCP) $(OCPFLAGS) $< $@
	$(OD) $(ODFLAGS) $< > $(BUILDDIR)/$(TARGET_SECURE).lst

$(BUILDDIR)/$(TARGET_SECURE).bin: $(BUILDDIR)/$(TARGET_SECURE)_0xC100.bin
	@echo "Generating secure image $@"
	$(PYTHON) ./tools/create_cust_image_blob.py --bin $< --load-address 0xC000 --magic-num 0xC0 --version 0x0 --kek 8 --authkey 8 --encalgo 1 --authalgo 1 --authB 1 --authI 1 -o $(BUILDDIR)/$(TARGET_SECURE)
	$(PYTHON) ./tools/create_cust_wireupdate_blob.py --load-address 0x20000 --bin $(BUILDDIR)/$(TARGET_SECURE).bin -i 6 -o $(BUILDDIR)/$(TARGET_SECURE)_wire --options 0x1

$(BUILDDIR)/$(TARGET_OTA).bin: $(BUILDDIR)/$(TARGET).bin
	@echo "Generating OTA image $@"
	$(PYTHON) ./tools/create_cust_image_blob.py --bin $< --load-address 0xc000 --magic-num 0xcb -o $(BUILDDIR)/$(TARGET_OTA)_temp --version $(TARGET_VERSION)
	$(PYTHON) ./tools/ota_binary_converter.py --appbin $(BUILDDIR)/$(TARGET_OTA)_temp.bin -o $(BUILDDIR)/$(TARGET_OTA)
	@$(CP) $(BUILDDIR)/$(TARGET_OTA)_temp.bin $(BUILDDIR)/$(TARGET_OTA)_LoRaWAN.bin
	@$(RM) -rf $(BUILDDIR)/$(TARGET_OTA)_temp.bin

$(BUILDDIR)/$(TARGET_WIRE).bin: $(BUILDDIR)/$(TARGET).bin
	@echo "Generating UART wire image $@"
	$(PYTHON) ./tools/create_cust_image_blob.py --bin $< --load-address 0xc000 --magic-num 0xcb -o $(BUILDDIR)/$(TARGET_WIRE)_temp --version 0x0
	$(PYTHON) ./tools/create_cust_wireupdate_blob.py --load-address 0x20000 --bin $(BUILDDIR)/$(TARGET_WIRE)_temp.bin -i 6 -o $(BUILDDIR)/$(TARGET_WIRE) --options 0x1
	@$(RM) -rf $(BUILDDIR)/$(TARGET_WIRE)_temp.bin


clean:
	@echo "Cleaning..."
	$(RM) -f $(OBJS) $(DEPS) $(BUILDDIR)/$(TARGET).a
	$(RM) -rf $(BUILDDIR)
	$(MAKE) -C $(BSP_DIR) AMBIQ_SDK=$(AMBIQ_SDK) clean

$(BUILDDIR)/%.d: ;

-include $(DEPS)

