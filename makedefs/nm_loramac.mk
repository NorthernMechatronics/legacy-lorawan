ifndef NM_SDK
    $(error NM SDK location NM_SDK not defined)
endif

INCLUDES += -I$(LORAMAC)/src/mac
INCLUDES += -I$(LORAMAC)/src/mac/region
INCLUDES += -I$(LORAMAC)/src/boards
INCLUDES += -I$(LORAMAC)/src/radio
INCLUDES += -I$(LORAMAC)/src/system
INCLUDES += -I$(LORAMAC)/src/apps/LoRaMac/common
INCLUDES += -I$(NM_SDK)/features/loramac-node/src/apps/LoRaMac/common/LmHandler/packages
INCLUDES += -I$(NM_SDK)/features/loramac-node/src/apps/LoRaMac/common/LmHandler

ifdef DEBUG
    LIBS += -lloramac-dev
else
    LIBS += -lloramac
endif
