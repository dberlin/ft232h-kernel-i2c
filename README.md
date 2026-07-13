# ft232h_i2c

Linux kernel driver exposing an FTDI **FT232H** as a standard I2C master via
its MPSSE engine. Registers an `i2c_adapter`, so `/dev/i2c-N`, `i2cdetect`,
and in-kernel I2C client drivers work normally.

## Wiring (external pull-ups required)

| FT232H pin | I2C signal            |
|------------|-----------------------|
| ADBUS0     | SCL                   |
| ADBUS1     | SDA (tie to ADBUS2)   |
| ADBUS2     | SDA (tie to ADBUS1)   |

Add ~4.7 kΩ pull-ups from SCL and SDA to VCC. ADBUS1 and ADBUS2 must be tied
together (SDA is driven on AD1 and sensed on AD2).

## Build & load (quick)

    make
    sudo insmod ft232h_i2c.ko            # default 100 kHz
    sudo insmod ft232h_i2c.ko speed=400000   # 400 kHz

## Install (persistent)

Into the running kernel's module tree:

    make
    sudo make modules_install            # -> /lib/modules/$(uname -r)/updates
    sudo modprobe ft232h_i2c

### DKMS (auto-rebuild on kernel upgrades, signs with the DKMS MOK)

    sudo cp -r . /usr/src/ft232h_i2c-1.0
    sudo dkms add    ft232h_i2c/1.0
    sudo dkms build  ft232h_i2c/1.0
    sudo dkms install ft232h_i2c/1.0

### Load at boot

    sudo cp ft232h_i2c-modules-load.conf /etc/modules-load.d/ft232h_i2c.conf

## The ftdi_sio conflict

The stock `ftdi_sio` serial driver also matches the FT232H (0403:6014) and
autoloads, usually claiming the interface first.

**The driver handles this itself:** at load time `ft232h_i2c` walks the USB bus
and, for any FT232H held by another driver, unbinds it and binds itself
(look for `taking over from ftdi_sio` in `dmesg`). So a plain `modprobe
ft232h_i2c` (or the boot-load above) is enough — no manual unbinding.

The in-kernel takeover runs once at load, so it does not cover **hot-plugging
the FT232H while the module is already loaded** (ftdi_sio can grab the fresh
device). For that case, install the bundled udev rule, which hands the device
over on every bind:

    sudo cp 99-ft232h-i2c.rules /etc/udev/rules.d/
    sudo udevadm control --reload

Manual one-off fix (if you ever need it):

    echo -n 3-4:1.0 | sudo tee /sys/bus/usb/drivers/ftdi_sio/unbind
    echo -n 3-4:1.0 | sudo tee /sys/bus/usb/drivers/ft232h_i2c/bind
    # (replace 3-4:1.0 with your device's interface; see ls /sys/bus/usb/drivers/ftdi_sio/)

## Use

    ls /sys/bus/i2c/devices/i2c-*/name   # find the "FT232H MPSSE I2C" adapter
    sudo i2cdetect -y N
    sudo i2cget -y N 0x50 0x00

## Limitations

- Standard (100 kHz) / fast (400 kHz) modes only; single master.
- No 10-bit addressing, no clock stretching (SCL is push-pull driven).
- Requires external pull-ups.
