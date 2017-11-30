obj-$(CONFIG_CAN_CORE) += can-core.o
obj-$(CONFIG_CAN_MCP251X)	+= mcp251x.o
obj-$(CONFIG_CAN_MCP2515)   += mcp2515.o

ifeq ($(CONFIG_CAN_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
