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
	u8  *cmdbuf;    /* assembled MPSSE command stream for one transfer */
	u8  *replybuf;  /* collected reply bytes for one transfer */
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

/* MPSSE command-stream fragment sizes, in bytes emitted per I2C primitive. */
#define FRAG_START  9
#define FRAG_STOP   12
#define FRAG_RW     12   /* one write-byte or read-byte (data + ACK bit) */

/*
 * Largest transfer we batch into a single bulk-OUT/bulk-IN pair, counted in
 * reply bytes (one per address or data byte). The command buffer is sized for
 * the worst case where every reply byte carries its own START.
 */
#define FT232H_MAX_REPLY  256
#define FT232H_CMD_CAP    (FT232H_MAX_REPLY * (FRAG_RW + FRAG_START) + \
			   FRAG_STOP + 1)

static int ftdi_ctrl(struct ft232h_i2c *priv, u8 request, u16 value)
{
	return usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
			       request,
			       USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			       value, priv->index, NULL, 0,
			       FTDI_USB_TIMEOUT_MS);
}

/*
 * Send an MPSSE command stream of arbitrary length. The chip concatenates
 * successive bulk-OUT transfers into its command FIFO, so splitting a long
 * stream across chunks is safe (a write->write boundary does not race the
 * MPSSE engine the way a read->write boundary does).
 */
static int ftdi_write(struct ft232h_i2c *priv, const u8 *buf, int len)
{
	int off = 0;

	while (off < len) {
		int chunk = min(len - off, 512);
		int ret, actual;

		memcpy(priv->txbuf, buf + off, chunk);
		ret = usb_bulk_msg(priv->udev,
				   usb_sndbulkpipe(priv->udev, priv->ep_out),
				   priv->txbuf, chunk, &actual,
				   FTDI_USB_TIMEOUT_MS);
		if (ret)
			return ret;
		if (actual != chunk)
			return -EIO;
		off += chunk;
	}
	return 0;
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

/*
 * Command-stream builder. An entire I2C transfer (every START, address, data
 * byte and its ACK, plus the final STOP) is assembled into one MPSSE command
 * buffer, sent with a single bulk-OUT, and its replies collected with a single
 * bulk-IN. Doing one byte per USB round-trip instead raced the MPSSE engine:
 * issuing a command bulk-OUT immediately after the previous reply's bulk-IN
 * occasionally dropped the leading command byte, desyncing the transfer and
 * returning stale shift-register data. Batching removes that read->write
 * boundary from inside a transfer entirely.
 */
struct i2c_cmdbuf {
	u8  *buf;
	int  len;
	int  nreply;   /* reply bytes the chip will produce for this stream */
};

static void emit(struct i2c_cmdbuf *c, const u8 *b, int n)
{
	memcpy(c->buf + c->len, b, n);
	c->len += n;
}

static void emit_start(struct i2c_cmdbuf *c)
{
	static const u8 s[FRAG_START] = {
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, DIR_WRITE, /* both high */
		MPSSE_SET_LOW_BYTE, PIN_SCL,            DIR_WRITE, /* SDA low   */
		MPSSE_SET_LOW_BYTE, 0x00,               DIR_WRITE, /* SCL low   */
	};
	emit(c, s, sizeof(s));
}

static void emit_stop(struct i2c_cmdbuf *c)
{
	static const u8 s[FRAG_STOP] = {
		MPSSE_SET_LOW_BYTE, 0x00,               DIR_WRITE, /* both low  */
		MPSSE_SET_LOW_BYTE, PIN_SCL,            DIR_WRITE, /* SCL high  */
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, DIR_WRITE, /* SDA high  */
		MPSSE_SET_LOW_BYTE, PIN_SCL | PIN_SDAO, 0x00,      /* release   */
	};
	emit(c, s, sizeof(s));
}

/* Clock out one byte, release SDA, then clock in the slave's ACK bit. */
static void emit_write_byte(struct i2c_cmdbuf *c, u8 val)
{
	u8 s[FRAG_RW] = {
		MPSSE_SET_LOW_BYTE, 0x00, DIR_WRITE,   /* SCL low, drive SDA   */
		MPSSE_BYTES_OUT_NEG, 0x00, 0x00, val,  /* clock out 1 byte     */
		MPSSE_SET_LOW_BYTE, 0x00, DIR_READ,    /* release SDA for ACK  */
		MPSSE_BITS_IN_POS, 0x00,               /* clock in 1 ACK bit   */
	};
	emit(c, s, sizeof(s));
	c->nreply++;
}

/* Clock in one byte, then drive the master's ACK (0) or NACK (1) bit. */
static void emit_read_byte(struct i2c_cmdbuf *c, bool send_ack)
{
	u8 s[FRAG_RW] = {
		MPSSE_SET_LOW_BYTE, 0x00, DIR_READ,    /* SCL low, SDA released */
		MPSSE_BYTES_IN_POS, 0x00, 0x00,        /* clock in 1 byte       */
		MPSSE_SET_LOW_BYTE, 0x00, DIR_WRITE,   /* drive SDA for ACK bit */
		MPSSE_BITS_OUT_NEG, 0x00,
			send_ack ? 0x00 : 0x80,        /* MSB-first: 0=ACK, 0x80=NACK */
	};
	emit(c, s, sizeof(s));
	c->nreply++;
}

static int ft232h_master_xfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	struct ft232h_i2c *priv = i2c_get_adapdata(adap);
	struct i2c_cmdbuf c = { .buf = priv->cmdbuf };
	u8 send_imm = MPSSE_SEND_IMMEDIATE;
	int ret, i, j, ridx, need = 0;

	/* Whole transfer must fit our command/reply buffers (one bulk-IN). */
	for (i = 0; i < num; i++)
		need += 1 + msgs[i].len;        /* address + payload = replies */
	if (need > FT232H_MAX_REPLY)
		return -EINVAL;

	mutex_lock(&priv->io_lock);

	/* Assemble the entire transfer as one command stream. */
	for (i = 0; i < num; i++) {
		struct i2c_msg *m = &msgs[i];
		u8 addr = (m->addr << 1) | ((m->flags & I2C_M_RD) ? 1 : 0);

		emit_start(&c);
		emit_write_byte(&c, addr);
		if (m->flags & I2C_M_RD)
			for (j = 0; j < m->len; j++)
				emit_read_byte(&c, j != m->len - 1);
		else
			for (j = 0; j < m->len; j++)
				emit_write_byte(&c, m->buf[j]);
	}
	emit_stop(&c);
	emit(&c, &send_imm, 1);

	ret = ftdi_write(priv, c.buf, c.len);
	if (ret)
		goto out;
	ret = ftdi_read(priv, priv->replybuf, c.nreply);
	if (ret)
		goto out;

	/*
	 * Replies arrive in emission order, one byte per address/data byte.
	 * A 1-bit ACK read lands in bit 0 (upper bits are shift-register
	 * garbage); bit 0 clear means the slave ACKed.
	 */
	ridx = 0;
	for (i = 0; i < num; i++) {
		struct i2c_msg *m = &msgs[i];

		if (priv->replybuf[ridx++] & 0x01) {   /* address NAKed */
			ret = -ENXIO;
			goto out;
		}
		if (m->flags & I2C_M_RD) {
			for (j = 0; j < m->len; j++)
				m->buf[j] = priv->replybuf[ridx++];
		} else {
			for (j = 0; j < m->len; j++)
				if (priv->replybuf[ridx++] & 0x01) {
					ret = -EIO;    /* slave NAKed data */
					goto out;
				}
		}
	}
	ret = num;
out:
	mutex_unlock(&priv->io_lock);
	return ret;
}

static u32 ft232h_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*
 * Cap a single message so its address byte plus payload fit the reply buffer;
 * the core splits larger reads/writes into multiple transfers for us.
 */
static const struct i2c_adapter_quirks ft232h_quirks = {
	.max_read_len  = FT232H_MAX_REPLY - 1,
	.max_write_len = FT232H_MAX_REPLY - 1,
};

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
	/*
	 * TCK = 30MHz / (1 + div); 3-phase clocking then stretches each bit to
	 * 1.5 TCK periods, so SCL = TCK / 1.5. This divisor already targets
	 * TCK = 1.5 * speed, i.e. SCL = speed.
	 */
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
	priv->cmdbuf = devm_kmalloc(&intf->dev, FT232H_CMD_CAP, GFP_KERNEL);
	priv->replybuf = devm_kmalloc(&intf->dev, FT232H_MAX_REPLY, GFP_KERNEL);
	if (!priv->txbuf || !priv->rxbuf || !priv->cmdbuf || !priv->replybuf) {
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
	priv->adapter.quirks = &ft232h_quirks;
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

/*
 * The stock ftdi_sio serial driver also matches the FT232H and usually binds
 * first. For each FT232H interface currently held by another driver, release
 * it and bind ourselves. Called once at module load, after usb_register() has
 * already claimed any interface that was free.
 */
static int ft232h_take_over(struct usb_device *udev, void *unused)
{
	struct usb_host_config *cfg = udev->actconfig;
	struct usb_interface *intf;
	int i;

	if (le16_to_cpu(udev->descriptor.idVendor) != FT232H_VID ||
	    le16_to_cpu(udev->descriptor.idProduct) != FT232H_PID || !cfg)
		return 0;

	for (i = 0; i < cfg->desc.bNumInterfaces; i++) {
		intf = cfg->interface[i];
		if (!intf || intf->dev.driver == &ft232h_i2c_driver.driver)
			continue;
		if (intf->dev.driver) {
			dev_info(&intf->dev, "taking over from %s\n",
				 intf->dev.driver->name);
			device_release_driver(&intf->dev);
		}
		if (device_driver_attach(&ft232h_i2c_driver.driver, &intf->dev))
			dev_warn(&intf->dev, "could not bind ft232h_i2c\n");
	}
	return 0;
}

static int __init ft232h_i2c_init(void)
{
	int ret = usb_register(&ft232h_i2c_driver);

	if (ret)
		return ret;
	usb_for_each_dev(NULL, ft232h_take_over);
	return 0;
}
module_init(ft232h_i2c_init);

static void __exit ft232h_i2c_exit(void)
{
	usb_deregister(&ft232h_i2c_driver);
}
module_exit(ft232h_i2c_exit);

MODULE_AUTHOR("Danny Berlin");
MODULE_DESCRIPTION("FTDI FT232H I2C master (MPSSE over USB)");
MODULE_LICENSE("GPL");
