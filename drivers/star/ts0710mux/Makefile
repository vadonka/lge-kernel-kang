#LGE_KERNEL_MUX START
EXTRA_CFLAGS += -DLGE_KERNEL_MUX
ifeq ($(TARGET_MODEM),mdm)
obj-y += ts0710_mux.o 
endif
ifeq ($(TARGET_MODEM),ifx)
obj-y += ts0710_mux_ifx.o
endif
#LGE_KERNEL_MUX END

