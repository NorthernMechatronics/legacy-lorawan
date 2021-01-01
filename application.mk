#******************************************************************************
#
# Step 1
# Define the locations of the various SDKs and libraries.
#
#******************************************************************************
NM_SDK    := $(shell pwd)/../nmsdk
AMBIQ_SDK := $(shell pwd)/../AmbiqSuite-R2.5.1
FREERTOS  := $(shell pwd)/../FreeRTOS/FreeRTOS
CORDIO    := $(shell pwd)/../AmbiqSuite-R2.5.1/third_party/cordio
UECC      := $(shell pwd)/../AmbiqSuite-R2.5.1/third_party/uecc
LORAMAC   := $(shell pwd)/../LoRaMac-node

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
# Include additional defines, source, header, libraries or paths below.
#
# Examples:
#   DEFINES  += -Dadditional_define
#   INCLUDES += -Iadditional_include_path
#   VPATH    += additional_source_path
#   LIBS     += -ladditional_library
#******************************************************************************
DEFINES += -DSOFT_SE
#DEFINES += -DSECURE_ELEMENT_PRE_PROVISIONED

INCLUDES += -I$(NM_SDK)/platform/console
INCLUDES += -I./soft-se

VPATH += $(NM_SDK)/platform/console
VPATH += ./soft-se

SRC += console_task.c
SRC += gpio_service.c
SRC += iom_service.c

# secure element
SRC += aes.c
SRC += cmac.c
SRC += soft-se.c
SRC += soft-se-hal.c

SRC += application.c
SRC += application_cli.c
