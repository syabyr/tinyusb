include ../../../hw/bsp/family_support.mk

LOCAL_TI_DIR := $(TOP)/$(EXAMPLE_PATH)

INC += \
	. \
	$(LOCAL_TI_DIR) \
	$(LOCAL_TI_DIR)/inc \
	$(LOCAL_TI_DIR)/driverlib

CFLAGS += -DPART_LM4F120H5QR

EXAMPLE_SOURCE += \
	usb2can.c \
	usb2can_can.c \
	usb_descriptors.c

SRC_C += $(addprefix $(EXAMPLE_PATH)/, $(EXAMPLE_SOURCE))

SRC_C += \
	$(EXAMPLE_PATH)/driverlib/can.c \
	$(EXAMPLE_PATH)/driverlib/gpio.c \
	$(EXAMPLE_PATH)/driverlib/interrupt.c \
	$(EXAMPLE_PATH)/driverlib/sysctl.c

include ../../../hw/bsp/family_rules.mk