// SPDX-License-Identifier: GPL-2.0
/*
 * FTDI FT232H I2C master driver (MPSSE over USB bulk).
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/i2c.h>

#define FT232H_VID 0x0403
#define FT232H_PID 0x6014

struct ft232h_i2c {
	struct usb_device   *udev;
	struct usb_interface *intf;
	u8   ep_in;
	u8   ep_out;
	u16  index;
	int  maxpacket;
	struct i2c_adapter adapter;
	struct mutex io_lock;
	u8  *txbuf;
	u8  *rxbuf;
};

static int ft232h_i2c_probe(struct usb_interface *intf,
			    const struct usb_device_id *id)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct ft232h_i2c *priv;
	int ret;

	ret = usb_find_common_endpoints(alt, &ep_in, &ep_out, NULL, NULL);
	if (ret) {
		dev_err(&intf->dev, "bulk endpoints not found: %d\n", ret);
		return ret;
	}

	priv = devm_kzalloc(&intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	priv->intf = intf;
	priv->ep_in = ep_in->bEndpointAddress;
	priv->ep_out = ep_out->bEndpointAddress;
	priv->index = alt->desc.bInterfaceNumber + 1;
	priv->maxpacket = usb_endpoint_maxp(ep_in);
	mutex_init(&priv->io_lock);

	priv->txbuf = devm_kmalloc(&intf->dev, 512, GFP_KERNEL);
	priv->rxbuf = devm_kmalloc(&intf->dev, priv->maxpacket, GFP_KERNEL);
	if (!priv->txbuf || !priv->rxbuf) {
		usb_put_dev(priv->udev);
		return -ENOMEM;
	}

	usb_set_intfdata(intf, priv);

	dev_info(&intf->dev,
		 "FT232H bound: ep_in=0x%02x ep_out=0x%02x maxpacket=%d index=%u\n",
		 priv->ep_in, priv->ep_out, priv->maxpacket, priv->index);
	return 0;
}

static void ft232h_i2c_disconnect(struct usb_interface *intf)
{
	struct ft232h_i2c *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	usb_put_dev(priv->udev);
	dev_info(&intf->dev, "FT232H disconnected\n");
}

static const struct usb_device_id ft232h_i2c_ids[] = {
	{ USB_DEVICE(FT232H_VID, FT232H_PID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, ft232h_i2c_ids);

static struct usb_driver ft232h_i2c_driver = {
	.name       = "ft232h_i2c",
	.id_table   = ft232h_i2c_ids,
	.probe      = ft232h_i2c_probe,
	.disconnect = ft232h_i2c_disconnect,
};
module_usb_driver(ft232h_i2c_driver);

MODULE_AUTHOR("Danny Berlin");
MODULE_DESCRIPTION("FTDI FT232H I2C master (MPSSE over USB)");
MODULE_LICENSE("GPL");
