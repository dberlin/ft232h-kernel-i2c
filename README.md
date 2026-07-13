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

## The ftdi_sio conflict (important)

The stock `ftdi_sio` serial driver also matches the FT232H (0403:6014) and
autoloads, claiming the interface before `ft232h_i2c` can. Symptom: the module
loads but no I2C adapter appears and the interface is bound to `ftdi_sio`.

Install the bundled udev rule to hand the device over automatically (survives
reboots and replug):

    sudo cp 99-ft232h-i2c.rules /etc/udev/rules.d/
    sudo udevadm control --reload

One-off manual fix without the rule:

    echo -n 3-4:1.0 | sudo tee /sys/bus/usb/drivers/ftdi_sio/unbind
    echo -n 3-4:1.0 | sudo tee /sys/bus/usb/drivers/ft232h_i2c/bind
    # (replace 3-4:1.0 with your device's interface, see: ls /sys/bus/usb/drivers/ftdi_sio/)

## Use

    ls /sys/bus/i2c/devices/i2c-*/name   # find the "FT232H MPSSE I2C" adapter
    sudo i2cdetect -y N
    sudo i2cget -y N 0x50 0x00

## Limitations

- Standard (100 kHz) / fast (400 kHz) modes only; single master.
- No 10-bit addressing, no clock stretching (SCL is push-pull driven).
- Requires external pull-ups.
