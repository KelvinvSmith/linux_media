mk610-objs	:= mk610_core.o mk610_cards.o mk610_i2c.o mk610_dvb.o mk610_dma.o

obj-$(CONFIG_DVB_MK610) += mk610.o

ccflags-y += -Idrivers/media/tuners
ccflags-y += -Idrivers/media/dvb-core
ccflags-y += -Idrivers/media/dvb-frontends

