/*
 * Copyright (C) 2017-2018 Gotech
 * xinhui.xie@gotechcn.cn
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "mk610.h" 

char *gDataBuffer = NULL;           
char *gBDBuffer = NULL; 
dma_addr_t gDataBufferHW;
dma_addr_t gBDBufferHW;

int hh[8]={0};


void sg_dma_reg_init(struct mk610_dev *dev) 
{
	u32 ctrl_reg;
	ctrl_reg = pci_read(SG_DMA_BASE, SG_DMA_REG_CONTROL);
	ctrl_reg &= ~SG_DMA_CR_COALESCE_MAX;
	ctrl_reg |= BD_NUM << SG_DMA_CR_COALESCE_SHIFT;

	ctrl_reg |= SG_DMA_XR_IRQ_ALL_MASK; 
	ctrl_reg |= SG_DMA_CYCLIC_MASK;
	pci_write(SG_DMA_BASE, SG_DMA_REG_CONTROL, ctrl_reg);
	dev_info(&dev->pci_dev->dev, "dma OK in sg_dma_reg_init\n");
}



static int sg_dma_init_chan_bd(struct sg_dma_channel *chan) 
{
	int i;
	INIT_LIST_HEAD(&chan->free_seg_list);
	
	for (i = 0; i < SG_PACKETS; i++) {   //sg                                        
			chan->seg_v[i].hw.next_desc = AXI_PCIE_SG_ADDR + SEG_SIZE * ((i + 1) % (SG_PACKETS))+
				//SEG_SIZE*SG_BUF_PACKETS*nr;		
				((chan->seg_p - gBDBufferHW) & 0xFFFFFFFF); 
			
		
		chan->seg_v[i].hw.buf_addr = AXI_PCIE_DATA_ADDR + TS_PACKET_SIZE * i + 
				//(SG_DMA_PAGE_SIZE + 0x100)*nr;
				((chan->dma_addr - gDataBufferHW) & 0xFFFFFFFF);
				

		chan->seg_v[i].hw.control = TS_PACKET_SIZE & SG_DMA_BD_BUFFER_LEN; 
		chan->seg_v[i].hw.control |=  SG_DMA_BD_SOF_EOF; 
		//chan->seg_v[i].hw.status = 0 ; //+++

		list_add_tail(&chan->seg_v[i].node, &chan->free_seg_list); 
	}

	return 0;
}


static void sg_dma_channel_desc_cleanup(struct sg_dma_tx_descriptor *seg_v)  
{
	int i;

	for(i = 1; i<=BD_NUM;  i++) {
		memset(&seg_v->hw.status, 0, 4); 
		seg_v--;
	}
}


static void sg_dma_start(struct sg_dma_channel *chan) 
{
	int err = 0;
	u32 val, ctrl_reg;
	
	struct mk610_dev *dev = chan->dev;
	
	ctrl_reg = pci_read(SG_DMA_BASE, SG_DMA_REG_CONTROL);
	ctrl_reg |= SG_DMA_CR_RUNSTOP_MASK;
	pci_write(SG_DMA_BASE, SG_DMA_REG_CONTROL, ctrl_reg);

	/* Wait for the hardware to start */
	err = sg_dma_poll_timeout(chan->dev, SG_DMA_REG_STATUS, val,
				      !(val & SG_DMA_SR_HALTED_MASK), 10,
				      SG_DMA_LOOP_COUNT);

}

static inline void sg_dma_refresh(struct mk610_dev *dev)
{
	u32 i, k, kk, value;
	struct mk610_adapter *adapter 	= dev->adapter;
	struct sg_dma_tx_descriptor *seg_v;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter->dma.buf_cnt = ((adapter->dma.buf_cnt >=8) ? 0: adapter->dma.buf_cnt);	//%
		k = adapter->dma.buf_cnt;
		adapter->dma.reach_tail = false;
		seg_v = &(adapter->dma.seg_v[BD_NUM*(k+1) - 1]);
		if(seg_v->hw.status & 0x8c000000) {
						
			sg_dma_channel_desc_cleanup(seg_v); 

			adapter->dma.reach_tail = true;
			adapter->dma.buf_cnt++;
			
			if (adapter->dma.cnt < 3) 
				adapter->dma.cnt++;
		}
		adapter++; 						
	}
		
}


static int sg_dma_channel_reset(struct mk610_dev *dev)
{
	int err = 0;
	u32 val, ctrl_reg;

	ctrl_reg = pci_read(SG_DMA_BASE, SG_DMA_REG_CONTROL);
	pci_write(SG_DMA_BASE, SG_DMA_REG_CONTROL, ctrl_reg |
		       SG_DMA_CR_RESET_MASK);

	/* Wait for the hardware to finish reset */
	err = sg_dma_poll_timeout(dev, SG_DMA_REG_CONTROL, val,
				      !(val & SG_DMA_CR_RESET_MASK), 10,
				      SG_DMA_LOOP_COUNT);

	if (err) {
		dev_err(&dev->pci_dev->dev, "reset timeout, cr %x, sr %x\n",
			pci_read(SG_DMA_BASE, SG_DMA_REG_CONTROL),
			pci_read(SG_DMA_BASE, SG_DMA_REG_STATUS));
		return -EBUSY;
	}

	//chan->err = false;
	printk(KERN_INFO"__sg_dma_channel_reset__\n"); 
	return err;
}

static unsigned int sg_buffer[8] = {0, 255, 510, 765, 1020, 1275, 1530, 1785};
static void replace_tasklet_schedule(struct mk610_dev *dev) 
{
	struct mk610_adapter *adapter = dev->adapter;
	
	u8* data;
	u8 tid,k;
	int i;
	
	struct sg_dma_tx_descriptor *seg_v;
		
	spin_lock(&dev->adap_lock);		
	
	k = adapter->dma.buf_cnt & 0x07;
	data = adapter->dma.buf[k];

	for(i = 0; i < TS_NUM; i++) {
		tid = data[0]&0x07;
		data[0] = 0x47;	
	
		dvb_dmx_swfilter_packets(&(dev->adapter[tid].demux), data, 1); 
		data =  data + 192;

	}
	adapter->dma.buf_cnt++;
	spin_unlock(&dev->adap_lock);	
}

/**
 * sg_dma_irq_handler - DMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the Xilinx DMA channel structure
 * Note: called by 
 * Return: IRQ_HANDLED/IRQ_NONE
 */
int sg_dma_irq_process(struct mk610_dev *dev, u32 status)
{	
	


	if (status & SG_DMA_XR_IRQ_IOC_MASK) {
		replace_tasklet_schedule(dev);	
	 }

return_irq:
	return 0;
		
}

void sg_dma_enable(struct mk610_adapter *adap) 
{
	adap->dma.tasklet_on =true;
	u32 status, control;
	struct mk610_dev *dev = adap->dev;
	status = pci_read(SG_DMA_BASE, SG_DMA_REG_STATUS);
	control = pci_read(SG_DMA_BASE, SG_DMA_REG_CONTROL);
	printk(KERN_INFO"feed SG_DMA_REG_CONTROL:%08x,SG_DMA_REG_STATUS:%08x\n",control,status); 
	
}

void sg_dma_disable(struct mk610_adapter *adap) 
{
	adap->dma.tasklet_on =false;

}

void sg_dma_enable_tasklet(struct mk610_adapter *adap) 
{
	struct sg_dma_tx_descriptor *head_desc, *tail_desc;

	struct sg_dma_channel *chan1,*chan2;
	struct mk610_dev *dev = adap->dev; 
	int num= adap->nr;
	int invalid_num= (num+2)%8;
	chan1 = &adap->dma;
	chan2 = &dev->adapter[invalid_num].dma;

	if (chan1->err)
		return;
	spin_lock_irq(&adap->adap_lock);//+++


	tail_desc = list_last_entry(&chan1->free_seg_list,
				     struct sg_dma_tx_descriptor, node);
	head_desc = list_first_entry(&chan2->free_seg_list,
				    struct sg_dma_tx_descriptor, node);
	
	pci_write(SG_DMA_BASE, SG_DMA_MCRX_CDESC(adap->nr), (tail_desc->hw.next_desc));

	sg_dma_start(chan1); 
	dev_info(&dev->pci_dev->dev, "run in sg_dma_enable()\n");
	if (chan1->err)
		return;


	pci_write(SG_DMA_BASE, SG_DMA_MCRX_TDESC(adap->nr),  (head_desc)->hw.next_desc );
	spin_unlock_irq(&adap->adap_lock);

}




void sg_dma_free(struct mk610_dev *dev) 
{
	int i;

	pci_free_consistent(dev->pci_dev, 
			2<<20, gBDBuffer, gBDBufferHW);
	pci_free_consistent(dev->pci_dev, 
			2<<21, gDataBuffer, gDataBufferHW);
	gBDBuffer = NULL;
	gDataBuffer = NULL;
	
	for (i = 0; i < 8; i++) 
		INIT_LIST_HEAD(&dev->adapter[i].dma.free_seg_list);
	dev_info(&dev->pci_dev->dev, "envoke sg_dma_free()\n");
}

static void axi_pci_trans(struct mk610_dev *dev)
{
	size_t pntr0, pntr1;
	
	pntr0 =  (size_t) (gBDBufferHW);
	pntr1 =  (size_t) (gDataBufferHW);	
    pci_write (MK610_PCIE_BASE, AXIBAR2PCIEBAR_0L, (pntr0 >> 0)  & 0xFFFFFFFF); 
    pci_write (MK610_PCIE_BASE, AXIBAR2PCIEBAR_0U, (pntr0 >> 32) & 0xFFFFFFFF); 
    pci_write (MK610_PCIE_BASE, AXIBAR2PCIEBAR_1L, (pntr1 >> 0)  & 0xFFFFFFFF); 
    pci_write (MK610_PCIE_BASE, AXIBAR2PCIEBAR_1U, (pntr1 >> 32) & 0xFFFFFFFF); 
	
}

int sg_dma_init(struct mk610_dev *dev) 
{
	struct mk610_adapter *adapter = dev->adapter; 
	int i, j;
	
	
	
	gDataBuffer = pci_alloc_consistent(dev->pci_dev, 
			2<<21, &gDataBufferHW); 
		
	
	gBDBuffer = pci_alloc_consistent(dev->pci_dev, 
			2<<20, &gBDBufferHW); 

	if (NULL == gDataBuffer || NULL == gBDBuffer)
		goto err;			
					  
	for (i = 0; i < dev->info->adapters; i++) {	

		adapter->dma.buf[0] = gDataBuffer + (SG_DMA_PAGE_SIZE + 0x100)*i;
		adapter->dma.cnt = 0;
		for (j = 1; j < SG_DMA_BUFFERS + 1; j++)
			adapter->dma.buf[j] = adapter->dma.buf[j-1] + SG_DMA_BUF_SIZE;

		adapter->dma.seg_v = gBDBuffer + SEG_SIZE*SG_PACKETS*i; 
		adapter->dma.seg_p = gBDBufferHW + SEG_SIZE*SG_PACKETS*i;
		
		
		adapter->dma.dma_addr = (gDataBufferHW + (SG_DMA_PAGE_SIZE + 0x100)*i);
		
		dev_info(&dev->pci_dev->dev, "dma enter sg_dma_init_chan_bd\n");		
		sg_dma_init_chan_bd(&adapter->dma);
		
		adapter->dma.buf_cnt = 0; 
		adapter->dma.offset = 0; 
		adapter->dma.reach_tail = false; 
		adapter->dma.err = false; 
		spin_lock_init(&adapter->adap_lock);
		adapter++;
	}
	dev_info(&dev->pci_dev->dev, "dma OK before sg_dma_channel_reset\n");
	sg_dma_channel_reset(dev);
	sg_dma_reg_init(dev);
	spin_lock_init(&dev->adap_lock);
	
	axi_pci_trans(dev);
	dev_info(&dev->pci_dev->dev, "dma OK after axi_pci_trans\n");	

	return 0;
err:
	dev_err(&dev->pci_dev->dev, "dma: memory alloc failed\n");
	return -ENOMEM;
}

