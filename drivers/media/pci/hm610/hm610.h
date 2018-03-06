/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _HM610_H_
#define _HM610_H_

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <linux/dmaengine.h>
#include <linux/version.h>

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/wait.h>   

#include "demux.h" 
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_net.h"
#include "dvbdev.h"
#include "hm610_regs.h"
#include "hm610_dma.h"

#define HM610_VID		0xC111
#define HM610_PID		0x0001

#define HM610_BOARD 		0x0610 

#define HM610_MAX_ADAPTERS	(8)


#define HM610_GPIODEF_NONE	(0)
#define HM610_GPIODEF_HIGH	(1)
#define HM610_GPIODEF_LOW	(2)

#define SG_DMA_BUFFERS	8


struct hm610_dev ; 



struct hm610_gpio_pin {
	u8 lvl;
	u8 nr; 
};

struct hm610_gpio_config {  
	struct hm610_gpio_pin lnb_power;  
	struct hm610_gpio_pin lnb_voltage;
	struct hm610_gpio_pin demod_reset;
};

struct hm610_adap_config {  
	u32 ts_in;
	u8 i2c_bus_nr;
	struct hm610_gpio_config gpio;
};

enum xilinx_i2c_state {
	STATE_DONE,
	STATE_ERROR,
	STATE_START
};

struct hm610_board {
	char *name;
	int adapters;
	u8 eeprom_i2c; 
	u8 eeprom_addr;
	struct hm610_adap_config adap_config[8];
};


struct hm610_i2c {
	struct hm610_dev 		*dev;
	struct i2c_adapter 		i2c_adap;
	struct mutex 			lock;
	wait_queue_head_t 		wq;
	struct i2c_msg			*tx_msg;
	unsigned int			tx_pos;
	unsigned int			nmsgs;
	enum xilinx_i2c_state		state;
	struct i2c_msg			*rx_msg;
	int				rx_pos;
};

struct sg_dma_channel {
	dma_addr_t dma_addr; 
	u8 *buf[SG_DMA_BUFFERS + 1];
	u8 offset;
	u8 cnt;	
	struct list_head free_seg_list;
	struct sg_dma_tx_descriptor *seg_v; 
	dma_addr_t seg_p; 
	bool reach_tail; 
	bool err;
	bool tasklet_on;
	struct hm610_dev *dev; 
	u8 buf_cnt; 
};

struct hm610_adapter {
	int nr;
	struct hm610_adap_config *cfg;
	struct hm610_dev *dev;
	struct hm610_i2c *i2c;
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dvb_net dvbnet;
	struct dmx_frontend fe_hw;
	struct dmx_frontend fe_mem;
	int feeds;
	spinlock_t adap_lock; 
	struct sg_dma_channel dma;
};

struct hm610_dev {
	struct hm610_board *info;
	struct pci_dev *pci_dev;
	void __iomem *lmmio;
	bool msi;
	struct hm610_adapter adapter[HM610_MAX_ADAPTERS];
	struct hm610_i2c i2c_bus;	
	struct tasklet_struct tasklet;	
	spinlock_t adap_lock; 

};

#define pci_read(_b, _o)	readl(dev->lmmio + (_b + _o))   
#define pci_write(_b, _o, _v)	writel((_v), dev->lmmio + (_b + _o))


void hm610_gpio_set_pin(struct hm610_dev *dev,
			  struct hm610_gpio_pin *pin, int state);


extern int hm610_i2c_init(struct hm610_dev *dev);
extern void hm610_i2c_exit(struct hm610_dev *dev);
extern int xiic_irq_process(struct hm610_i2c *i2c);
extern struct hm610_board hm610_boards[];
extern int hm610_dvb_init(struct hm610_adapter *adapter);
extern void hm610_dvb_exit(struct hm610_adapter *adapter);
extern int  sg_dma_init(struct hm610_dev *dev);
extern void sg_dma_free(struct hm610_dev *dev);
extern void sg_dma_reg_init(struct hm610_dev *dev);
extern void sg_dma_enable(struct hm610_adapter *adap);
extern void sg_dma_disable(struct hm610_adapter *adap);
extern int  sg_dma_irq_process(struct hm610_dev *dev, u32 status); 
extern void sg_dma_enable_tasklet(struct hm610_adapter *adap);

#define SEG_SIZE  sizeof(struct sg_dma_tx_descriptor) 

#endif



