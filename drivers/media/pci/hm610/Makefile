hm610-objs	:= hm610_core.o hm610_cards.o hm610_i2c.o hm610_dvb.o hm610_dma.o

obj-$(CONFIG_DVB_hm610) += hm610.o

ccflags-y += -Idrivers/media/tuners
ccflags-y += -Idrivers/media/dvb-core
ccflags-y += -Idrivers/media/dvb-frontends

