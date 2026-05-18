# MagGrad Dual V1

STM32H743 firmware for the dual magnetometer board.

## Build and Flash

```bash
make all
make flash
```

Serial debug output is sent to both USB CDC and USART1 at 115200 baud.

## Magnetometer Initialization Notes

AK09973D and TMAG3001 sensors sit behind TCA9548A I2C multiplexers. A downstream sensor branch can hold SDA low if the previous I2C transaction is interrupted, if a device is left mid-transfer, or if a channel is selected while the downstream device is not ready. When that happens, the upstream bus looks broken even though the TCA and other channels may still be fine.

Observed failure:

```text
TCA select OK, then SDA=0
HAL_I2C_GetError(...) = 0x00000020
subsequent TCA deselect/select calls timeout
```

The fix used in this firmware is:

1. Reset the sensor bus before a full scan.
2. Deselect all TCA channels.
3. Select exactly one TCA channel.
4. Check whether SDA is held low after selecting the channel.
5. If SDA is low, switch SCL/SDA to open-drain GPIO, send clock pulses, generate STOP, reinitialize I2C, reset the bus, and retry the TCA selection.
6. Probe the sensor ID before configuration.
7. Configure the sensor.
8. Deselect the TCA channel before moving to the next channel.

AK09973D is currently initialized with:

```text
CNTL1 = 0x0000
CNTL2 = 0x04
```

That means continuous 10 Hz mode, low-noise drive, high-sensitivity range. The 100 Hz setting previously made debugging less stable, so 10 Hz is the known-good baseline for repeated init/read testing.

TMAG3001 uses the same bus-recovery pattern on I2C3. I2C3 pins are:

```text
SCL = PA8
SDA = PC9
RESET = PC8
```

I2C3 PA8/PC9 must be configured as alternate-function open-drain with internal
pull-ups enabled. On 2026-05-18, TMAG3001 had previously read correctly, then
all downstream devices stopped ACKing after I2C3 GPIO was changed to
`GPIO_NOPULL`. Restoring `GPIO_PULLUP` on both PA8 and PC9 brought the bus back
immediately:

```text
TMAG: TCA OK
TMAG CH1: 3/3 OK
TMAG CH2: 3/3 OK
TMAG CH3: 3/3 OK
TMAG CH4: 3/3 OK
TMAG: first pass 12/12 OK
```

When PA8/PC9 were left as `GPIO_NOPULL`, the debug scan saw no TMAG3001 address
behind any mux channel and TCA select calls started failing. Treat that as an
I2C3 electrical/GPIO configuration failure first, not as a TMAG3001 register
configuration problem.

TMAG3001 is configured from standby, with continuous mode written last:

```text
Device_Config_1 = 0x00
Device_Config_2 = 0x00   before configuration
Sensor_Config_1 = 0x70   XYZ magnetic channels enabled
Sensor_Config_2 = 0x00   raw XYZ, default ranges, angle disabled
THR_Config_1..3 = 0x00
Sensor_Config_3 = 0x00
INT_Config_1 = 0x00      interrupts disabled
Sensor_Config_4..6 = 0x00
Device_Config_2 = 0x02   continuous conversion mode
```

After entering continuous mode, the driver polls `Conv_Status.Result_Status`
before accepting the sensor as initialized. The runtime firmware initializes AK
and TMAG once, then loops on reads.

Do not switch TMAG3001 `Device_Config_1.I2C_RD[1:0]` to the direct result-read
mode without a matching recovery path. In testing, direct-read mode caused later
manufacturer-ID register reads to return measurement-frame bytes instead of
`0x5449` until the firmware forced `Device_Config_1 = 0x00` at the start of
initialization. Keep that force-write in place so a previous bad configuration
does not survive a reset line pulse.

For stable full-frame serial output, the TMAG read path keeps a 10 ms mux-settle
delay after each TCA channel selection and a 5 ms delay after each successful
sensor read. Removing the per-sensor 5 ms delay raised the AK-only loop rate but
caused TMAG CSV rows to disappear in the current USB CDC/read timing setup.

AK09973D I2C1 pins are:

```text
SCL = PB8
SDA = PB9
RESET = PB7
```

AK09973D I2C2 pins are:

```text
SCL = PB10
SDA = PB11
RESET = PB12
```

Do not remove the bus-clear path unless the hardware is changed and long-duration serial testing confirms no downstream branch holds SDA low.
