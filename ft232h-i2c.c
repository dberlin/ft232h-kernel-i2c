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

static unsigned int speed = 100000;
module_param(speed, uint, 0444);
MODULE_PARM_DESC(speed, "I2C bus speed in Hz (default 100000; e.g. 400000)");

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

/* FTDI vendor control requests (bRequest) */
#define FTDI_REQ_RESET        0x00
#define FTDI_REQ_SET_LATENCY  0x09
#define FTDI_REQ_SET_BITMODE  0x0b
/* RESET wValue */
#define FTDI_RESET_SIO        0
#define FTDI_RESET_PURGE_RX   1
#define FTDI_RESET_PURGE_TX   2
/* BITMODE modes (high byte of wValue) */
#define FTDI_BITMODE_RESET    0x00
#define FTDI_BITMODE_MPSSE    0x02

/* MPSSE opcodes */
#define MPSSE_SET_LOW_BYTE    0x80  /* +value +direction */
#define MPSSE_READ_LOW_BYTE   0x81  /* returns 1 byte: ADBUS pin states */
#define MPSSE_BYTES_OUT_NEG   0x11  /* MSB first, out on falling edge */
#define MPSSE_BYTES_IN_POS    0x20  /* MSB first, in on rising edge */
#define MPSSE_BITS_OUT_NEG    0x13  /* MSB first, out on falling edge */
#define MPSSE_BITS_IN_POS     0x22  /* MSB first, in on rising edge */
#define MPSSE_SEND_IMMEDIATE  0x87
#define MPSSE_DIS_DIV5        0x8a
#define MPSSE_EN_3PHASE       0x8c
#define MPSSE_DIS_ADAPTIVE    0x97
#define MPSSE_SET_CLK_DIV     0x86
#define MPSSE_LOOPBACK_OFF    0x85
#define MPSSE_BAD_CMD         0xaa  /* provokes 0xFA sync marker */

/* ADBUS pin bits (value + direction bytes for SET_LOW_BYTE) */
#define PIN_SCL   BIT(0)   /* ADBUS0 */
#define PIN_SDAO  BIT(1)   /* ADBUS1 (SDA out) */
#define PIN_SDAI  BIT(2)   /* ADBUS2 (SDA in) */
#define DIR_WRITE (PIN_SCL | PIN_SDAO)  /* SCL + SDA driven */
#define DIR_READ  (PIN_SCL)             /* SCL driven, SDA released */

#define FTDI_USB_TIMEOUT_MS 1000

static int ftdi_ctrl(struct ft232h_i2c *priv, u8 request, u16 value)
{
	return usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
			       request,
			       USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			       value, priv->index, NULL, 0,
			       FTDI_USB_TIMEOUT_MS);
}

static int ftdi_write(struct ft232h_i2c *priv, const u8 *buf, int len)
{
	int ret, actual;

	if (len > 512)
		return -EINVAL;
	memcpy(priv->txbuf, buf, len);
	ret = usb_bulk_msg(priv->udev,
			   usb_sndbulkpipe(priv->udev, priv->ep_out),
			   priv->txbuf, len, &actual, FTDI_USB_TIMEOUT_MS);
	if (ret)
		return ret;
	return actual == len ? 0 : -EIO;
}

/* Read exactly @len MPSSE data bytes; strip 2 modem-status bytes per packet. */
static int ftdi_read(struct ft232h_i2c *priv, u8 *buf, int len)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(FTDI_USB_TIMEOUT_MS);
	int got = 0, ret, actual, n;

	while (got < len) {
		actual = 0;
		ret = usb_bulk_msg(priv->udev,
				   usb_rcvbulkpipe(priv->udev, priv->ep_in),
				   priv->rxbuf, priv->maxpacket, &actual,
				   FTDI_USB_TIMEOUT_MS);
		if (ret)
			return ret;
		if (actual <= 2) {           /* modem status only, no data */
			if (time_after(jiffies, deadline))
				return -ETIMEDOUT;
			continue;
		}
		n = min(actual - 2, len - got);
		memcpy(buf + got, priv->rxbuf + 2, n);
		got += n;
	}
	return 0;
}

/* Write a command stream, optionally read @rlen reply data bytes. */
static int mpsse_cmd(struct ft232h_i2c *priv, const u8 *cmd, int clen,
		     u8 *reply, int rlen)
{
	int ret = ftdi_write(priv, cmd, clen);

	if (ret)
		return ret;
	if (rlen)
		return ftdi_read(priv, reply, rlen);
	return 0;
}

static int i2c_start(struct ft232h_i2c *priv)
{
	u8 cmd[] = {
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, DIR_WRITE, /* both high */
		MPSSE_SET_LOW_BYTE, PIN_SCL,            DIR_WRITE, /* SDA low   */
		MPSSE_SET_LOW_BYTE, 0x00,               DIR_WRITE, /* SCL low   */
	};
	return mpsse_cmd(priv, cmd, sizeof(cmd), NULL, 0);
}

static int i2c_stop(struct ft232h_i2c *priv)
{
	u8 cmd[] = {
		MPSSE_SET_LOW_BYTE, 0x00,               DIR_WRITE, /* both low  */
		MPSSE_SET_LOW_BYTE, PIN_SCL,            DIR_WRITE, /* SCL high  */
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, DIR_WRITE, /* SDA high  */
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, 0x00,      /* release   */
	};
	return mpsse_cmd(priv, cmd, sizeof(cmd), NULL, 0);
}

static int i2c_write_byte(struct ft232h_i2c *priv, u8 val, bool *ack)
{
	u8 reply = 0;
	int ret;
	u8 cmd[] = {
		MPSSE_SET_LOW_BYTE, 0x00, DIR_WRITE,   /* SCL low, drive SDA   */
		MPSSE_BYTES_OUT_NEG, 0x00, 0x00, val,  /* clock out 1 byte     */
		MPSSE_SET_LOW_BYTE, 0x00, DIR_READ,    /* release SDA for ACK  */
		MPSSE_BITS_IN_POS, 0x00,               /* clock in 1 ACK bit   */
		MPSSE_SEND_IMMEDIATE,
	};

	ret = mpsse_cmd(priv, cmd, sizeof(cmd), &reply, 1);
	if (ret)
		return ret;
	/* Sub-byte reads land in the MSBs; SDA low (bit clear) == ACK. */
	*ack = !(reply & 0x80);
	return 0;
}

static int i2c_read_byte(struct ft232h_i2c *priv, u8 *val, bool send_ack)
{
	u8 reply = 0;
	int ret;
	u8 cmd[] = {
		MPSSE_SET_LOW_BYTE, 0x00, DIR_READ,    /* SCL low, SDA released */
		MPSSE_BYTES_IN_POS, 0x00, 0x00,        /* clock in 1 byte       */
		MPSSE_SET_LOW_BYTE, 0x00, DIR_WRITE,   /* drive SDA for ACK bit */
		MPSSE_BITS_OUT_NEG, 0x00,
			send_ack ? 0x00 : 0x80,        /* MSB-first: 0=ACK,0x80=NACK */
		MPSSE_SEND_IMMEDIATE,
	};

	ret = mpsse_cmd(priv, cmd, sizeof(cmd), &reply, 1);
	if (ret)
		return ret;
	*val = reply;
	return 0;
}

static int ft232h_xfer_msg(struct ft232h_i2c *priv, struct i2c_msg *msg)
{
	bool ack;
	int ret, i;
	u8 addr = (msg->addr << 1) | ((msg->flags & I2C_M_RD) ? 1 : 0);

	ret = i2c_start(priv);
	if (ret)
		return ret;

	ret = i2c_write_byte(priv, addr, &ack);
	if (ret)
		return ret;
	if (!ack)
		return -ENXIO;               /* no device at this address */

	if (msg->flags & I2C_M_RD) {
		for (i = 0; i < msg->len; i++) {
			ret = i2c_read_byte(priv, &msg->buf[i],
					    i != msg->len - 1);
			if (ret)
				return ret;
		}
	} else {
		for (i = 0; i < msg->len; i++) {
			ret = i2c_write_byte(priv, msg->buf[i], &ack);
			if (ret)
				return ret;
			if (!ack)
				return -EIO;   /* slave NACKed data */
		}
	}
	return 0;
}

static int ft232h_master_xfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	struct ft232h_i2c *priv = i2c_get_adapdata(adap);
	int ret = 0, i;

	mutex_lock(&priv->io_lock);
	for (i = 0; i < num; i++) {
		ret = ft232h_xfer_msg(priv, &msgs[i]);
		if (ret)
			break;
	}
	i2c_stop(priv);
	mutex_unlock(&priv->io_lock);

	return ret ? ret : num;
}

static u32 ft232h_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ft232h_algo = {
	.master_xfer   = ft232h_master_xfer,
	.functionality = ft232h_func,
};

static int mpsse_init(struct ft232h_i2c *priv, u32 speed_hz)
{
	u16 div;
	u8 reply[4];
	int ret;
	u8 sync[] = { MPSSE_BAD_CMD };
	u8 cfg[] = {
		MPSSE_DIS_DIV5,
		MPSSE_DIS_ADAPTIVE,
		MPSSE_EN_3PHASE,
		MPSSE_LOOPBACK_OFF,
		MPSSE_SET_CLK_DIV, 0, 0,       /* divisor filled below */
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, 0x00, /* idle: released high */
	};

	if (speed_hz == 0 || speed_hz > 20000000)
		return -EINVAL;
	div = (20000000 / speed_hz) - 1;
	cfg[5] = div & 0xff;
	cfg[6] = (div >> 8) & 0xff;

	ret = ftdi_ctrl(priv, FTDI_REQ_RESET, FTDI_RESET_SIO);
	if (ret)
		return ret;
	ret = ftdi_ctrl(priv, FTDI_REQ_SET_LATENCY, 16);
	if (ret)
		return ret;
	ret = ftdi_ctrl(priv, FTDI_REQ_SET_BITMODE,
			(FTDI_BITMODE_RESET << 8) | 0x00);
	if (ret)
		return ret;
	ret = ftdi_ctrl(priv, FTDI_REQ_SET_BITMODE,
			(FTDI_BITMODE_MPSSE << 8) | 0x00);
	if (ret)
		return ret;
	ftdi_ctrl(priv, FTDI_REQ_RESET, FTDI_RESET_PURGE_RX);
	ftdi_ctrl(priv, FTDI_REQ_RESET, FTDI_RESET_PURGE_TX);

	/* Sync: bad opcode must echo back the 0xFA error marker. */
	ret = ftdi_write(priv, sync, sizeof(sync));
	if (ret)
		return ret;
	ret = ftdi_read(priv, reply, 2);
	if (ret)
		return ret;
	if (reply[0] != 0xfa) {
		dev_err(&priv->intf->dev,
			"MPSSE sync failed (got 0x%02x 0x%02x)\n",
			reply[0], reply[1]);
		return -EIO;
	}

	ret = ftdi_write(priv, cfg, sizeof(cfg));
	if (ret)
		return ret;

	dev_info(&priv->intf->dev, "MPSSE synced, I2C clock %u Hz (div=%u)\n",
		 speed_hz, div);

	/* Bus health: SCL (AD0) and SDA-in (AD2) should idle high via pull-ups. */
	{
		u8 rd[] = { MPSSE_READ_LOW_BYTE, MPSSE_SEND_IMMEDIATE };
		u8 pins = 0;

		ret = ftdi_write(priv, rd, sizeof(rd));
		if (ret)
			return ret;
		ret = ftdi_read(priv, &pins, 1);
		if (ret)
			return ret;
		if (!(pins & PIN_SCL) || !(pins & PIN_SDAI))
			dev_warn(&priv->intf->dev,
				 "I2C lines not idle-high (pins=0x%02x); check pull-ups and AD1=AD2 wiring\n",
				 pins);
		else
			dev_info(&priv->intf->dev,
				 "I2C bus idle OK (pins=0x%02x)\n", pins);
	}
	return 0;
}

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

	ret = mpsse_init(priv, speed);
	if (ret) {
		usb_put_dev(priv->udev);
		return ret;
	}

	i2c_set_adapdata(&priv->adapter, priv);
	priv->adapter.owner = THIS_MODULE;
	priv->adapter.algo = &ft232h_algo;
	priv->adapter.dev.parent = &intf->dev;
	strscpy(priv->adapter.name, "FT232H MPSSE I2C",
		sizeof(priv->adapter.name));

	ret = i2c_add_adapter(&priv->adapter);
	if (ret) {
		usb_put_dev(priv->udev);
		return ret;
	}
	dev_info(&intf->dev, "registered %s\n", priv->adapter.name);
	return 0;
}

static void ft232h_i2c_disconnect(struct usb_interface *intf)
{
	struct ft232h_i2c *priv = usb_get_intfdata(intf);

	i2c_del_adapter(&priv->adapter);
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
