#******************************************************************************
#
# Step 1
# Define the locations of the various SDKs and libraries.
#
#******************************************************************************
NM_SDK    := $(PWD)/../nmsdk
AMBIQ_SDK := $(PWD)/../AmbiqSuite-R2.5.1
FREERTOS  := $(PWD)/../FreeRTOS/FreeRTOS
CORDIO    := $(PWD)/../AmbiqSuite-R2.5.1/third_party/cordio
UECC      := $(PWD)/../AmbiqSuite-R2.5.1/third_party/uecc
LORAMAC   := $(PWD)/../LoRaMac-node

#******************************************************************************
#
# Step 2
# Specify the location of the board support package to be used.
#
#******************************************************************************
BSP_DIR := $(NM_SDK)/bsp/nm180100evb

#******************************************************************************
#
# Step 3
# Specify output target name
#
#******************************************************************************
ifdef DEBUG
    TARGET   := lorawan-dev
else
    TARGET   := lorawan
endif

#******************************************************************************
#
# Step 4
# Include additional source, header, libraries or paths below.
#
# Examples:
#   INCLUDES += -Iadditional_include_path
#   VPATH    += additional_source_path
#   LIBS     += -ladditional_library
#******************************************************************************

INCLUDES += -I$(NM_SDK)/platform/console
INCLUDES += -I$(NM_SDK)/platform/lorawan
INCLUDES += -I$(NM_SDK)/platform/lorawan/LmHandler
INCLUDES += -I$(NM_SDK)/platform/lorawan/LmHandler/packages
INCLUDES += -I./soft-se

VPATH += $(NM_SDK)/platform/console
VPATH += $(NM_SDK)/platform/lorawan
VPATH += $(NM_SDK)/platform/lorawan/LmHandler
VPATH += $(NM_SDK)/platform/lorawan/LmHandler/packages
VPATH += ./soft-se

SRC += console_task.c
SRC += gpio_service.c
SRC += iom_service.c

# secure element
SRC += aes.c
SRC += cmac.c
SRC += soft-se.c
SRC += soft-se-hal.c

# LoRaMAC Application Layer Handler
SRC += NvmCtxMgmt.c
SRC += LmHandler.c
SRC += LmHandlerMsgDisplay.c
SRC += FragDecoder.c
SRC += LmhpClockSync.c
SRC += LmhpCompliance.c
SRC += LmhpFragmentation.c
SRC += LmhpRemoteMcastSetup.c

SRC += application.c
