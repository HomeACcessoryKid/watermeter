PROGRAM = main

EXTRA_COMPONENTS = \
    extras/i2s_dma \
    $(abspath UDPlogger) \

FLASH_SIZE ?= 8

ifdef VERSION
EXTRA_CFLAGS += -DVERSION=\"$(VERSION)\"
endif

EXTRA_CFLAGS += -DUDPLOG_PRINTF_TO_UDP

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud $(ESPBAUD) --elf $(PROGRAM_OUT)
