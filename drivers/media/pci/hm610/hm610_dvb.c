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
#include "mxl58x.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static u32 GPIO_VALUE = 0xFFFFFF00; 

struct sec_priv {
	struct hm610_adapter *adap;
	int (*set_voltage)(struct dvb_frontend *fe,
			   enum fe_sec_voltage voltage);
};

static int hm610_set_voltage(struct dvb_frontend* fe,
		enum fe_sec_voltage voltage)
{
	struct sec_priv *priv = fe->sec_priv;
	struct hm610_gpio_config *cfg = &priv->adap->cfg->gpio;
	struct hm610_dev *dev = priv->adap->dev;

	dev_dbg(&dev->pci_dev->dev, "%s() %s\n", __func__,
		voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
		voltage == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" :
		"SEC_VOLTAGE_OFF");

	switch (voltage) {
		case SEC_VOLTAGE_13:
			hm610_gpio_set_pin(dev, &cfg->lnb_power, 1);
			hm610_gpio_set_pin(dev, &cfg->lnb_voltage, 0);
			break;
		case SEC_VOLTAGE_18:
			hm610_gpio_set_pin(dev, &cfg->lnb_power, 1);
			hm610_gpio_set_pin(dev, &cfg->lnb_voltage, 1);
			break;
		default: 
			break;
	}

	if (priv->set_voltage)
		return priv->set_voltage(fe, voltage);
	else
		return 0;
}

static void hm610_release_sec(struct dvb_frontend* fe)
{
	struct sec_priv *priv;

	if (fe == NULL)
		return;

	priv = fe->sec_priv;
	if (priv == NULL)
		return;

	fe->ops.set_voltage = priv->set_voltage; 
	fe->sec_priv = NULL;
	kfree(priv);
}

static struct dvb_frontend *hm610_attach_sec(struct hm610_adapter *adap, struct dvb_frontend *fe)
{
	struct sec_priv *priv;

	priv = kzalloc(sizeof(struct sec_priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	priv->set_voltage = fe->ops.set_voltage;
	priv->adap = adap;

	fe->ops.set_voltage = hm610_set_voltage;
	fe->sec_priv = priv;

	return fe;
}

static int set_mac_address(struct hm610_adapter *adap) 
{
	struct hm610_dev *dev = adap->dev;

	struct i2c_adapter *i2c = &dev->i2c_bus.i2c_adap;
	u8 eep_addr[2]; 
	int ret;

	struct i2c_msg msg[] = {
		{ .addr = 0x50, .flags = 0,
		  .buf = eep_addr, .len = 2 },
		{ .addr = 0x50, .flags = I2C_M_RD,
		  .buf = adap->dvb_adapter.proposed_mac, .len = 6 }
	};
	eep_addr[0] = 0x00;
	if (dev->info->eeprom_addr)	
		eep_addr[1] = dev->info->eeprom_addr; 
	else
		eep_addr[1] = 0xa0;
	eep_addr[1] += 0x10 * adap->nr;
	ret = i2c_transfer(i2c, msg, 2);
	if (ret != 2) {
		dev_warn(&dev->pci_dev->dev,
			"error reading MAC address for adapter %d\n",
			adap->nr);
	} else {
		dev_info(&dev->pci_dev->dev,
			"MAC address %pM\n", adap->dvb_adapter.proposed_mac);
	}
	return 0;
};

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct hm610_adapter *adapter = dvbdmx->priv;

	if (!adapter->feeds)
		sg_dma_enable(adapter);
	printk(KERN_INFO"__start_feed__\n"); 
	return ++adapter->feeds;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct hm610_adapter *adapter = dvbdmx->priv;

	if (--adapter->feeds)
		return adapter->feeds;

	sg_dma_disable(adapter);
	return 0;
}


static int max_set_voltage(struct i2c_adapter *i2c,
		enum fe_sec_voltage voltage, u8 rf_in)
{
	struct hm610_i2c *i2c_adap = i2c_get_adapdata(i2c);
	struct hm610_dev *dev = i2c_adap->dev;


	if (rf_in > 3)
		return -EINVAL;

	switch (voltage) {
	case SEC_VOLTAGE_13:
		GPIO_VALUE |= HM610_GPIO_PIN(rf_in, 0); 
		GPIO_VALUE &= ~HM610_GPIO_PIN(rf_in, 1);
		break;
	case SEC_VOLTAGE_18:
		GPIO_VALUE |= HM610_GPIO_PIN(rf_in, 0);
		GPIO_VALUE |= HM610_GPIO_PIN(rf_in, 1);
		break;
	case SEC_VOLTAGE_OFF:
	default:

		GPIO_VALUE |= ~HM610_GPIO_PIN(rf_in, 0);
		break;
	}

	pci_write(HM610_GPIO_BASE, 0, GPIO_VALUE);
	return 0;
}



static struct mxl58x_cfg hm610_mxl58x_cfg = {
	.adr		= 0x60,
	.type		= 0x01,
	.clk		= 24000000,
	.cap		= 12,
	.fw_read	= NULL,

	.set_voltage	= max_set_voltage,
};

static int hm610_frontend_attach(struct hm610_adapter *adapter)
{
	struct hm610_dev *dev = adapter->dev;
	struct pci_dev *pci = dev->pci_dev;


	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;

	adapter->fe = NULL;


	set_mac_address(adapter);

	switch (pci->subsystem_vendor) {
	case 0x0610:


		adapter->fe = dvb_attach(mxl58x_attach, i2c,
				&hm610_mxl58x_cfg, adapter->nr);
		if (adapter->fe == NULL)
			goto frontend_atach_fail;

		if (hm610_attach_sec(adapter, adapter->fe) == NULL) {
			dev_warn(&dev->pci_dev->dev,
				"error attaching lnb control on adapter %d\n",
				adapter->nr);
		}

		break;
	default:
		dev_warn(&dev->pci_dev->dev, "unknonw card\n");
		return -ENODEV;
		break;
	}
	strlcpy(adapter->fe->ops.info.name,hm610_boards[pci->subsystem_vendor].name,52);
	return 0;

frontend_atach_fail:

	if (adapter->fe != NULL)
		dvb_frontend_detach(adapter->fe);
	adapter->fe = NULL;
	dev_err(&dev->pci_dev->dev, "hm610 frontend %d attach failed\n",
		adapter->nr);

	return -ENODEV;
}

int hm610_dvb_init(struct hm610_adapter *adapter)
{
	struct hm610_dev *dev = adapter->dev;
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;
	struct dmxdev *dmxdev;
	struct dmx_frontend *fe_hw;
	struct dmx_frontend *fe_mem;
	int ret;

	ret = dvb_register_adapter(adap, "hm610 DVB Adapter",
					THIS_MODULE,
					&adapter->dev->pci_dev->dev,
					adapter_nr);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "error registering adapter\n");
		if (ret == -ENFILE)
			dev_err(&dev->pci_dev->dev,
				"increase DVB_MAX_ADAPTERS (%d)\n",
				DVB_MAX_ADAPTERS);
		return ret;
	}

	adap->priv = adapter;
	dvbdemux->priv = adapter;
	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);

	ret = dvb_dmx_init(dvbdemux);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_dmx_init failed\n");
		goto err0;
	}

	dmxdev = &adapter->dmxdev;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;

	ret = dvb_dmxdev_init(dmxdev, adap);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_dmxdev_init failed\n");
		goto err1;
	}

	fe_hw = &adapter->fe_hw;
	fe_mem = &adapter->fe_mem;

	fe_hw->source = DMX_FRONTEND_0;
	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_hw);
	if ( ret < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_dmx_init failed");
		goto err2;
	}

	fe_mem->source = DMX_MEMORY_FE;
	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_mem);
	if (ret  < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_dmx_init failed");
		goto err3;
	}

	ret = dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, fe_hw);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_dmx_init failed");
		goto err4;
	}

	ret = dvb_net_init(adap, &adapter->dvbnet, adapter->dmxdev.demux);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "dvb_net_init failed");
		goto err5;
	}

	hm610_frontend_attach(adapter);
	if (adapter->fe == NULL) {
		dev_err(&dev->pci_dev->dev, "frontend attach failed\n");
		ret = -ENODEV;
		goto err6;
	}

	ret = dvb_register_frontend(adap, adapter->fe);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "frontend register failed\n");
		goto err7;
	}

	return ret;

err7:
	dvb_frontend_detach(adapter->fe);
err6:
	hm610_release_sec(adapter->fe);

	dvb_net_release(&adapter->dvbnet);
err5:
	dvbdemux->dmx.close(&dvbdemux->dmx);
err4:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_mem);
err3:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_hw);
err2:
	dvb_dmxdev_release(dmxdev);
err1:
	dvb_dmx_release(dvbdemux);
err0:
	dvb_unregister_adapter(adap);
	return ret;
}

void hm610_dvb_exit(struct hm610_adapter *adapter)
{
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;

	if (adapter->fe) {
		dvb_unregister_frontend(adapter->fe);
		hm610_release_sec(adapter->fe);
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;

	}
	dvb_net_release(&adapter->dvbnet);
	dvbdemux->dmx.close(&dvbdemux->dmx);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &adapter->fe_mem);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &adapter->fe_hw);
	dvb_dmxdev_release(&adapter->dmxdev);
	dvb_dmx_release(&adapter->demux);
	dvb_unregister_adapter(adap);
}
