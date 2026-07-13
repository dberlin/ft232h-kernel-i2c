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

## Build & load

    make
    sudo insmod ft232h_i2c.ko            # default 100 kHz
    sudo insmod ft232h_i2c.ko speed=400000   # 400 kHz

`ftdi_sio` must not hold the device. If it does:

    sudo modprobe -r ftdi_sio   # or unbind the specific interface

## Use

    ls /sys/bus/i2c/devices/i2c-*/name   # find the "FT232H MPSSE I2C" adapter
    sudo i2cdetect -y N
    sudo i2cget -y N 0x50 0x00

## Limitations

- Standard (100 kHz) / fast (400 kHz) modes only; single master.
- No 10-bit addressing, no clock stretching (SCL is push-pull driven).
- Requires external pull-ups.
