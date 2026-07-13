# FT232H I2C Kernel Driver — Design

**Date:** 2026-07-13
**Status:** Approved
**Goal:** A working out-of-tree Linux kernel module that binds to an FTDI FT232H
over USB, drives its MPSSE engine as an I2C master, and exposes the bus through
the standard kernel I2C subsystem (`/dev/i2c-N`, `i2cdetect`, in-kernel client
drivers).

## Target environment

- Kernel 7.1.3 (Fedora), kernel build headers present at
  `/lib/modules/$(uname -r)/build`.
- Device: FTDI FT232H, USB ID `0403:6014`, single interface `:1.0` with two
  bulk endpoints — **EP 0x02 OUT**, **EP 0x81 IN**.
- No driver currently claims the interface; `ftdi_sio` is not loaded, so binding
  is unobstructed. (If `ftdi_sio` is ever loaded and grabs the device, it must be
  unbound first — documented, not worked around.)
- `i2c-dev` loaded; toolchain present (gcc 16, make 4.4).

## Scope

**In scope:** I2C master only, standard-mode (100 kHz) default with a configurable
speed, exposed as a Linux `i2c_adapter`.

**Explicitly out of scope (YAGNI for "working driver for my own use"):**
- GPIO / SPI cell exposure (no MFD split).
- 10-bit addressing.
- Clock-stretch detection.
- Multi-master arbitration.
- Async URB pipelining.

These are documented as limitations; they can be added later without reworking
the core.

## Approach

A single standalone `usb_driver` module, `ft232h-i2c`, that registers an
`i2c_adapter` directly. Rejected alternatives:

- **MFD split** (mfd core + i2c/gpio/spi platform cells, the upstream
  `ft232h-intf` shape): more indirection than an I2C-only personal driver needs.
- **i2c-tiny-usb-style firmware shim**: not applicable; the FT232H runs fixed
  MPSSE, not custom loadable firmware.

## Module structure

- `ft232h-i2c.c` — USB probe/disconnect, MPSSE init, I2C algorithm. Single
  focused file (~400–500 lines); split only if GPIO/SPI is ever added.
- `Makefile` (Kbuild) building against `/lib/modules/$(uname -r)/build`.

## USB layer

- `usb_driver` with `id_table = {{ USB_DEVICE(0x0403, 0x6014) }}`.
- **Probe:** locate bulk-in/bulk-out endpoints from the interface descriptor;
  allocate DMA-safe (`kmalloc`, `GFP_KERNEL`) transfer buffers; `usb_set_intfdata`;
  run MPSSE init; `i2c_add_adapter`.
- **Disconnect:** `i2c_del_adapter`, free resources.
- All bus traffic via synchronous `usb_bulk_msg` with a sane timeout — simple and
  correct at I2C speeds; no async URBs.
- FT232H setup via FTDI vendor control requests: reset (`SIO_RESET`), set latency
  timer (low, e.g. 1–16 ms), set bitmode to **MPSSE (0x02)** with an appropriate
  pin direction mask.

## MPSSE I2C engine

**Wiring (standard FTDI I2C, requires external pull-ups):**
- ADBUS0 = SCL
- ADBUS1 = SDA out
- ADBUS2 = SDA in (tie ADBUS1 + ADBUS2 together externally)

Open-drain is emulated by toggling pin **direction** (output = drive low,
input = released/pulled high by the external resistor) rather than driving a
logic high. Output value bits for SCL/SDA are held at 0.

**MPSSE configuration commands sent at init:**
- `0x8A` — disable clock divide-by-5 (use the 60 MHz master clock).
- `0x8C` — enable three-phase clocking (required for I2C; data valid on both
  edges).
- `0x97` — disable adaptive clocking.
- `0x86 <lo> <hi>` — set clock divisor.

**Clock divisor math** (with three-phase clocking):
`I2C_freq = 20 MHz / (1 + divisor)`.
- 100 kHz → divisor 199 (`0x00C7`)
- 400 kHz → divisor 49 (`0x0031`)

**Primitives** (built from `0x80` set-data-bits and `0x10`/`0x11`/`0x20`/`0x22`
clock-bytes/bits opcodes, with `0x87` "send immediate" to flush before reads):
- `i2c_start` — SDA high→low while SCL high, then SCL low.
- `i2c_stop` — SDA low, SCL high, SDA low→high.
- `i2c_write_byte` — clock out 8 bits MSB-first, then clock in one ACK bit;
  return ACK/NACK.
- `i2c_read_byte` — clock in 8 bits, then clock out one ACK (0) or NACK (1) bit.

## I2C algorithm (`master_xfer`)

- For each `struct i2c_msg` in the batch:
  - Emit START (repeated-START between messages; a single STOP after the last
    message of the batch).
  - Address byte = `(addr << 1) | (read ? 1 : 0)`; on address NACK return
    `-ENXIO`.
  - Data phase: writes stream bytes and check each ACK (NACK mid-write →
    `-EREMOTEIO`); reads clock in `len` bytes, NACKing the final byte.
- Batch the outgoing MPSSE command stream into as few bulk writes as practical,
  then read back returned bytes with a bulk read.
- `functionality` = `I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMULATED` (SMBus rides on top
  via the I2C core).

## Config & testing

- Module parameter `speed` (Hz, default `100000`) selects the clock divisor.
- **Test loop:**
  1. `make`
  2. `sudo insmod ft232h-i2c.ko` (optionally `speed=400000`)
  3. Confirm a new `/dev/i2c-N` appears; check `dmesg` for probe log.
  4. `i2cdetect -y N` — scan the bus.
  5. If a real slave is wired, read/write it (e.g. `i2cget`).
  6. `sudo rmmod ft232h_i2c` — confirm clean unload.

## Limitations (documented)

- 100 kHz default (configurable to 400 kHz; higher untested).
- No 10-bit addressing, no clock-stretch handling, single-master only.
- External pull-ups on SCL/SDA are required.
