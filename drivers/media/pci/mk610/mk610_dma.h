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
 
#ifndef _MK610_DMA_H_
#define _MK610_DMA_H_

#define BD_NUM  		255 
#define TS_NUM  		1020 
#define TS_PACKET_SIZE		768 
#define SG_DMA_BUFFERS		8
#define SG_DMA_PAGE_SIZE	(BD_NUM * SG_DMA_BUFFERS * TS_PACKET_SIZE) 
#define SG_DMA_BUF_SIZE	(SG_DMA_PAGE_SIZE / SG_DMA_BUFFERS) 
#define SG_PACKETS	2040
#define SG_BUF_PACKETS 255
struct sg_dma_desc_hw { 
	u32 next_desc;
	u32 pad1;
	u32 buf_addr;
	u32 pad2;
	u32 mcdma_fields;
	u32 vsize_stride;
	u32 control;
	u32 status;
} __aligned(64);

struct sg_dma_tx_descriptor { 
	struct sg_dma_desc_hw hw;
	dma_addr_t phys; 
	struct list_head node;
} __aligned(64);

#endif

