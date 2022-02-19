PROGRAM = main

EXTRA_COMPONENTS = \
	extras/rboot-ota \
	extras/timekeeping \
    extras/i2s_dma \
    extras/paho_mqtt_c \
    $(abspath UDPlogger) \
    $(abspath esp-adv-button) \

FLASH_SIZE ?= 8

ifdef VERSION
EXTRA_CFLAGS += -DVERSION=\"$(VERSION)\"
endif

BUTTON_PIN ?= 0
EXTRA_CFLAGS += -DBUTTON_PIN=$(BUTTON_PIN)

EXTRA_CFLAGS += -DUDPLOG_PRINTF_TO_UDP

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud $(ESPBAUD) --elf $(PROGRAM_OUT)

sig:
	openssl sha384 -binary -out firmware/main.bin.sig firmware/main.bin
	printf "%08x" `cat firmware/main.bin | wc -c`| xxd -r -p >>firmware/main.bin.sig
	ls -l firmware
