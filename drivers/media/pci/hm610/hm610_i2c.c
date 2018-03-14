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
 
#include "hm610.h"

#define xiic_tx_space(i2c) ((i2c)->tx_msg->len - (i2c)->tx_pos)
#define xiic_rx_space(i2c) ((i2c)->rx_msg->len - (i2c)->rx_pos)

static void __xiic_start_xfer(struct hm610_i2c *i2c);

static inline void xiic_irq_dis(struct hm610_i2c *i2c, u32 mask)
{
	struct hm610_dev *dev = i2c->dev;
	u32 ier = pci_read(HM610_I2C_BASE, HM610_IIER_OFFSET);
	pci_write(HM610_I2C_BASE, HM610_IIER_OFFSET, ier & ~mask);

}

static inline void xiic_irq_en(struct hm610_i2c *i2c, u32 mask)
{
	struct hm610_dev *dev = i2c->dev;
	u32 ier = pci_read(HM610_I2C_BASE, HM610_IIER_OFFSET);
	pci_write(HM610_I2C_BASE, HM610_IIER_OFFSET, ier | mask);
}

static inline void xiic_irq_clr(struct hm610_i2c *i2c, u32 mask)
{
	struct hm610_dev *dev = i2c->dev;
	u32 isr = pci_read(HM610_I2C_BASE, HM610_IISR_OFFSET);
	pci_write(HM610_I2C_BASE, HM610_IISR_OFFSET, isr & mask);
}

static inline void xiic_irq_clr_en(struct hm610_i2c *i2c, u32 mask)
{
	
	xiic_irq_clr(i2c, mask);
	xiic_irq_en(i2c, mask);
}

static void xiic_clear_rx_fifo(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	u8 sr;
	for (sr = pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET);
		!(sr & HM610_SR_RX_FIFO_EMPTY_MASK);
		sr = pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET))
		{
			pci_read(HM610_I2C_BASE, HM610_DRR_REG_OFFSET);
		}
}

static void xiic_reinit(struct hm610_i2c *i2c)
{

	struct hm610_dev *dev = i2c->dev;


	pci_write(HM610_I2C_BASE, HM610_RESETR_OFFSET, HM610_RESET_MASK);

	pci_write(HM610_I2C_BASE, HM610_RFD_REG_OFFSET, IIC_RX_FIFO_DEPTH - 1);

	pci_write(HM610_I2C_BASE, HM610_CR_REG_OFFSET, HM610_CR_TX_FIFO_RESET_MASK);

	pci_write(HM610_I2C_BASE, HM610_CR_REG_OFFSET, HM610_CR_ENABLE_DEVICE_MASK);

	xiic_clear_rx_fifo(i2c);

	pci_write(HM610_I2C_BASE, HM610_DGIER_OFFSET, HM610_GINTR_ENABLE_MASK);

	xiic_irq_clr_en(i2c, HM610_INTR_ARB_LOST_MASK);
}

static void xiic_deinit(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	u8 cr;

	pci_write(HM610_I2C_BASE, HM610_RESETR_OFFSET, HM610_RESET_MASK);

	cr = pci_read(HM610_I2C_BASE, HM610_CR_REG_OFFSET);

	pci_write(HM610_I2C_BASE, HM610_CR_REG_OFFSET, cr & ~HM610_CR_ENABLE_DEVICE_MASK);

}

static void xiic_read_rx(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	u8 bytes_in_fifo;
	int i;

	bytes_in_fifo = pci_read(HM610_I2C_BASE, HM610_RFO_REG_OFFSET) + 1;

	dev_dbg(i2c->i2c_adap.dev.parent,
		"%s entry, bytes in fifo: %d, msg: %d, SR: 0x%x, CR: 0x%x\n",
		__func__, bytes_in_fifo, xiic_rx_space(i2c),
		pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET),
		pci_read(HM610_I2C_BASE, HM610_CR_REG_OFFSET));

	if (bytes_in_fifo > xiic_rx_space(i2c))
		bytes_in_fifo = xiic_rx_space(i2c);

	for (i = 0; i < bytes_in_fifo; i++){
		i2c->rx_msg->buf[i2c->rx_pos++] = pci_read(HM610_I2C_BASE, HM610_DRR_REG_OFFSET);

	}
	pci_write(HM610_I2C_BASE, HM610_RFD_REG_OFFSET,
		(xiic_rx_space(i2c) > IIC_RX_FIFO_DEPTH) ?
		IIC_RX_FIFO_DEPTH - 1 :  xiic_rx_space(i2c) - 1);

}

static int xiic_tx_fifo_space(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;

	return IIC_TX_FIFO_DEPTH - pci_read(HM610_I2C_BASE, HM610_TFO_REG_OFFSET) - 1;
}

static void xiic_fill_tx_fifo(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;

	u8 fifo_space = xiic_tx_fifo_space(i2c);
	int len = xiic_tx_space(i2c);

	len = (len > fifo_space) ? fifo_space : len;

	dev_dbg(i2c->i2c_adap.dev.parent, "%s entry, len: %d, fifo space: %d\n",
		__func__, len, fifo_space);

	while (len--) {
		u16 data = i2c->tx_msg->buf[i2c->tx_pos++];
		if ((xiic_tx_space(i2c) == 0) && (i2c->nmsgs == 1)) {
			data |= HM610_TX_DYN_STOP_MASK;
			dev_dbg(i2c->i2c_adap.dev.parent, "%s TX STOP\n", __func__);
		}
		pci_write(HM610_I2C_BASE, HM610_DTR_REG_OFFSET, data);

	}
}

static void xiic_wakeup(struct hm610_i2c *i2c, int code)
{
	i2c->tx_msg = NULL;
	i2c->rx_msg = NULL;
	i2c->nmsgs = 0;
	i2c->state = code;
	wake_up(&i2c->wq);
}


int xiic_irq_process(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	u32 pend, isr, ier;
	u32 clr = 0;
	 
	mutex_lock(&i2c->lock);
	isr = pci_read(HM610_I2C_BASE, HM610_IISR_OFFSET);
	ier = pci_read(HM610_I2C_BASE, HM610_IIER_OFFSET);
	pend = isr & ier;

	dev_dbg(i2c->i2c_adap.dev.parent, "%s: IER: 0x%x, ISR: 0x%x, pend: 0x%x\n",
		__func__, ier, isr, pend);
	dev_dbg(i2c->i2c_adap.dev.parent, "%s: SR: 0x%x, msg: %p, nmsgs: %d\n",
		__func__, pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET),
		i2c->tx_msg, i2c->nmsgs);

	if ((pend & HM610_INTR_ARB_LOST_MASK) ||
		((pend & HM610_INTR_TX_ERROR_MASK) &&
		!(pend & HM610_INTR_RX_FULL_MASK))) {

		dev_dbg(i2c->i2c_adap.dev.parent, "%s error\n", __func__);

		xiic_reinit(i2c); 

		if (i2c->rx_msg)
			xiic_wakeup(i2c, STATE_ERROR);
		if (i2c->tx_msg)
			xiic_wakeup(i2c, STATE_ERROR);

	}
	if (pend & HM610_INTR_RX_FULL_MASK) {


		clr |= HM610_INTR_RX_FULL_MASK;
		if (!i2c->rx_msg) {
			dev_dbg(i2c->i2c_adap.dev.parent,
				"%s unexpexted RX IRQ\n", __func__);
			xiic_clear_rx_fifo(i2c);
			goto out;
		}

		xiic_read_rx(i2c);
		if (xiic_rx_space(i2c) == 0) {

			i2c->rx_msg = NULL;

			clr |= (isr & HM610_INTR_TX_ERROR_MASK);

			dev_dbg(i2c->i2c_adap.dev.parent,
				"%s end of message, nmsgs: %d\n",
				__func__, i2c->nmsgs);


			if (i2c->nmsgs > 1) {
				i2c->nmsgs--;
				i2c->tx_msg++;
				dev_dbg(i2c->i2c_adap.dev.parent,
					"%s will start next...\n", __func__);

				__xiic_start_xfer(i2c);
			}
		}

	}
	if (pend & HM610_INTR_BNB_MASK) {

		clr |= HM610_INTR_BNB_MASK;


		xiic_irq_dis(i2c, HM610_INTR_BNB_MASK);

		if (!i2c->tx_msg)
			goto out;

		if ((i2c->nmsgs == 1) && !i2c->rx_msg &&
			xiic_tx_space(i2c) == 0)
			xiic_wakeup(i2c, STATE_DONE);
		else
			xiic_wakeup(i2c, STATE_ERROR);

	}
	if (pend & (HM610_INTR_TX_EMPTY_MASK | HM610_INTR_TX_HALF_MASK)) {

		clr |= (pend &
			(HM610_INTR_TX_EMPTY_MASK | HM610_INTR_TX_HALF_MASK));

		if (!i2c->tx_msg) {
			dev_dbg(i2c->i2c_adap.dev.parent,
				"%s unexpexted TX IRQ\n", __func__);
			goto out;
		}

		xiic_fill_tx_fifo(i2c);


		if (!xiic_tx_space(i2c) && xiic_tx_fifo_space(i2c) >= 2) {

			dev_dbg(i2c->i2c_adap.dev.parent,
				"%s end of message sent, nmsgs: %d\n",
				__func__, i2c->nmsgs);

			if (i2c->nmsgs > 1) {
				i2c->nmsgs--;
				i2c->tx_msg++;

				__xiic_start_xfer(i2c);
			} else {
				xiic_irq_dis(i2c, HM610_INTR_TX_HALF_MASK);

				dev_dbg(i2c->i2c_adap.dev.parent,
					"%s Got TX IRQ but no more to do...\n",
					__func__);
			}
		} else if (!xiic_tx_space(i2c) && (i2c->nmsgs == 1)){
			xiic_irq_dis(i2c, HM610_INTR_TX_HALF_MASK);
			}

	}
out:
	dev_dbg(i2c->i2c_adap.dev.parent, "%s clr: 0x%x\n", __func__, clr);

	pci_write(HM610_I2C_BASE, HM610_IISR_OFFSET, clr);

	mutex_unlock(&i2c->lock);
	return 0;
}


static int xiic_busy(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	int tries = 3;
	int err = 1;
	u32 sr;

	while (err && tries--) {
		sr = pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET);
		err = (sr & HM610_SR_BUS_BUSY_MASK) ? -EBUSY : 0;
		msleep(1);
	}
	return err;
}

static void xiic_start_recv(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	u8 rx_watermark;
	struct i2c_msg *msg = i2c->rx_msg = i2c->tx_msg;


	xiic_irq_clr_en(i2c, HM610_INTR_RX_FULL_MASK | HM610_INTR_TX_ERROR_MASK);


	rx_watermark = msg->len;
	if (rx_watermark > IIC_RX_FIFO_DEPTH)
		rx_watermark = IIC_RX_FIFO_DEPTH;
	pci_write(HM610_I2C_BASE, HM610_RFD_REG_OFFSET, rx_watermark - 1);

	if (!(msg->flags & I2C_M_NOSTART))

		pci_write(HM610_I2C_BASE, HM610_DTR_REG_OFFSET,
			(msg->addr << 1) | HM610_READ_OPERATION |
			HM610_TX_DYN_START_MASK);

	xiic_irq_clr_en(i2c, HM610_INTR_BNB_MASK);

	pci_write(HM610_I2C_BASE, HM610_DTR_REG_OFFSET,
		msg->len | ((i2c->nmsgs == 1) ? HM610_TX_DYN_STOP_MASK : 0));

	if (i2c->nmsgs == 1)
		xiic_irq_clr_en(i2c, HM610_INTR_BNB_MASK);
	i2c->tx_pos = msg->len;
}

static void xiic_start_send(struct hm610_i2c *i2c)
{
	struct hm610_dev *dev = i2c->dev;
	struct i2c_msg *msg = i2c->tx_msg;

	xiic_irq_clr(i2c, HM610_INTR_TX_ERROR_MASK);

	dev_dbg(i2c->i2c_adap.dev.parent, "%s entry, msg: %p, len: %d",
		__func__, msg, msg->len);
	dev_dbg(i2c->i2c_adap.dev.parent, "%s entry, ISR: 0x%x, CR: 0x%x\n",
		__func__, pci_read(HM610_I2C_BASE, HM610_IISR_OFFSET),
		pci_read(HM610_I2C_BASE, HM610_CR_REG_OFFSET));

	if (!(msg->flags & I2C_M_NOSTART)) {
		u16 data = ((msg->addr << 1) & 0xfe) | HM610_WRITE_OPERATION |
			HM610_TX_DYN_START_MASK;
		if ((i2c->nmsgs == 1) && msg->len == 0)
			data |= HM610_TX_DYN_STOP_MASK;
		pci_write(HM610_I2C_BASE, HM610_DTR_REG_OFFSET, data);

	}

	xiic_fill_tx_fifo(i2c);

	xiic_irq_clr_en(i2c, HM610_INTR_TX_EMPTY_MASK | HM610_INTR_TX_ERROR_MASK |
		HM610_INTR_BNB_MASK);
}

static void __xiic_start_xfer(struct hm610_i2c *i2c)
{
	int first = 1;
	int fifo_space = xiic_tx_fifo_space(i2c);

	dev_dbg(i2c->i2c_adap.dev.parent, "%s entry, msg: %p, fifos space: %d\n",
		__func__, i2c->tx_msg, fifo_space);

	if (!i2c->tx_msg)
		return;

	i2c->rx_pos = 0;
	i2c->tx_pos = 0;
	i2c->state = STATE_START;
       while ((fifo_space >= 2) && (first || (i2c->nmsgs > 1))) {
                if (!first) {
                       i2c->nmsgs--;
                       i2c->tx_msg++;
                       i2c->tx_pos = 0;
                } else
                       first = 0;

                 if (i2c->tx_msg->flags & I2C_M_RD) {
                 
                         xiic_start_recv(i2c);
                        return;
                } else {
                        xiic_start_send(i2c);
                         if (xiic_tx_space(i2c) != 0) {
                               
                                break;
                        }
                 }

                 fifo_space = xiic_tx_fifo_space(i2c);
         }


	if (i2c->nmsgs > 1 || xiic_tx_space(i2c))
		xiic_irq_clr_en(i2c, HM610_INTR_TX_HALF_MASK);

}
#if 0
static void xiic_start_xfer(struct hm610_i2c *i2c)
{
	mutex_lock(&i2c->lock);
	xiic_reinit(i2c);
	__xiic_start_xfer(i2c);
	mutex_unlock(&i2c->lock);
}
#endif

static int xiic_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg *msgs, int num)
{

	struct hm610_i2c *i2c = i2c_get_adapdata(i2c_adap);
	struct hm610_dev *dev = i2c->dev;
	int err;

	dev_dbg(i2c_adap->dev.parent, "%s entry SR: 0x%x\n", __func__,
		pci_read(HM610_I2C_BASE, HM610_SR_REG_OFFSET));
		

	err = xiic_busy(i2c);
 	if (err)
		goto out; 

	i2c->tx_msg = msgs;
	i2c->nmsgs = num;
	mutex_lock(&i2c->lock); 
	xiic_reinit(i2c); 
	__xiic_start_xfer(i2c);
	mutex_unlock(&i2c->lock);
	if (wait_event_timeout(i2c->wq, (i2c->state == STATE_ERROR) || 
		(i2c->state == STATE_DONE), HZ)) { 
		err = (i2c->state == STATE_DONE) ? num : -EIO;
	} else {
		i2c->tx_msg = NULL;
		i2c->rx_msg = NULL;
		i2c->nmsgs = 0;
		err = -ETIMEDOUT;
		pr_err("i2c read error 2\n");

	}
	
out:
	
	return err;
}

static u32 i2c_functionality(struct i2c_adapter *i2c_adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm hm610_i2c_algo_template = {
	.master_xfer   = xiic_xfer,
	.functionality = i2c_functionality,
};

static int hm610_i2c_register(struct hm610_i2c *bus)
{
	struct hm610_dev *dev = bus->dev;
	struct i2c_adapter *i2c_adap;

	init_waitqueue_head(&bus->wq);
	mutex_init(&bus->lock);

	i2c_adap = &bus->i2c_adap;
	strcpy(i2c_adap->name, "hm610");
	i2c_adap->algo = &hm610_i2c_algo_template;
	i2c_adap->algo_data = (void*) bus;
	i2c_adap->dev.parent = &dev->pci_dev->dev;
	i2c_adap->owner = THIS_MODULE;
	i2c_set_adapdata(&bus->i2c_adap, bus);
	return i2c_add_adapter(&bus->i2c_adap);
}

static void hm610_i2c_unregister(struct hm610_i2c *bus)
{
	i2c_del_adapter(&bus->i2c_adap);
}

int hm610_i2c_init(struct hm610_dev *dev) 
{
	int ret = 0;

	dev->i2c_bus.dev = dev;
	ret = hm610_i2c_register(&dev->i2c_bus);
	if (ret) {
		hm610_i2c_unregister(&dev->i2c_bus);
		xiic_deinit(&dev->i2c_bus);//+++
	} else {
		xiic_reinit(&dev->i2c_bus); 
	}
	   
	return ret;
}

void hm610_i2c_exit(struct hm610_dev *dev)
{
	hm610_i2c_unregister(&dev->i2c_bus);
}
