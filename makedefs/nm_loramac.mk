ifndef NM_SDK
    $(error NM SDK location NM_SDK not defined)
endif

INCLUDES += -I$(LORAMAC)/src/mac
INCLUDES += -I$(LORAMAC)/src/mac/region
INCLUDES += -I$(LORAMAC)/src/boards
INCLUDES += -I$(LORAMAC)/src/radio
INCLUDES += -I$(LORAMAC)/src/system

ifdef DEBUG
    LIBS += -lloramac-dev
else
    LIBS += -lloramac
endif
