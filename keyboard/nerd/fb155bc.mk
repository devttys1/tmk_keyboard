FB155BC_DIR = fb155bc

SRC +=  serial_uart.c \
	fb155bc/fb155bc.c \
	fb155bc/fb155bc_task.c \
	fb155bc/main.c

OPT_DEFS += -DPROTOCOL_FB155BC

VPATH += $(FB155BC_DIR)
