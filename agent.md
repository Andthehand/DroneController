# Drone Controller Agent Notes

## Project Overview

This repository contains firmware for a custom drone controller built around a Raspberry Pi Pico 2 W using C/C++ and the Pico SDK.

When working in this repo, prefer small, testable firmware changes and keep hardware safety ahead of feature speed.

## Board And Tooling

- MCU: Raspberry Pi Pico 2 W
- Language: C/C++
- Build system: CMake + Pico SDK
- Board target in CMake: `pico2_w`
- Main firmware entrypoint: `Drone_Controller.c`

## Hardware Map

### BMI270 IMU

The BMI270 is connected on `spi0` and the current driver already expects this pinout:

- GPIO 2: SPI0 RX / MISO
- GPIO 3: SPI0 TX / MOSI
- GPIO 4: SPI0 SCK
- GPIO 5: BMI270 chip select

### ESC / Motor Outputs

SpeedyBee 55A ESC connections:

- GPIO 22: Motor 1, DSHOT300/600
- GPIO 21: Motor 2, DSHOT300/600
- GPIO 20: Motor 3, DSHOT300/600
- GPIO 19: Motor 4, DSHOT300/600
- GPIO 18: ESC current sense input
- GPIO 17: ESC telemetry input (ESC not supported)

Notes:

- Use PIO when communicating to the motors
- Treat motor-output code as safety-critical.
- Do not arm motors automatically on boot.
- Any DSHOT implementation should start with explicit failsafe behavior, disarmed defaults, and clear throttle bounds.
- If bidirectional DSHOT or RPM telemetry is added, document timing assumptions and DMA/PIO usage in code comments.
- SpeedyBee documentation for the matching F405 V4 BLS 55A stack lists ESC protocol support as `DSHOT300/600`.
- The same documentation lists ESC telemetry as `not supported`, so GPIO 17 should be treated as unused/reserved unless bench testing proves otherwise.

### Battery Measurement

- GPIO 27: battery voltage ADC input
- Voltage divider: 75k over 10k

Computed divider ratio assumptions:

- `Vadc = Vbattery * (10k / (75k + 10k))`
- `Vbattery = Vadc * 8.5`

Notes:

- Keep ADC scaling math explicit in code.
- Add filtering/calibration constants instead of hardcoding magic numbers where possible.
- SpeedyBee documentation for the matching stack lists current sensor settings of `scale = 400` and `offset = 0`, which is a useful baseline if current sensing from the ESC is implemented.

### Addressable LED

- GPIO 0: addressable `B3DK3BRG` LED

Notes:

- The LED part is `Harvatek B3DK3BRG-05C000113U1930`, an addressable single-wire RGB LED with integrated driver.
- Protocol details from the datasheet:
- Data rate: `800 kHz`
- Color order: `G`, `R`, `B` with 8 bits each, `24-bit` total per LED, `MSB first`
- Logic `0`: `0.3 us` high, then `0.9 us` low
- Logic `1`: `0.9 us` high, then `0.3 us` low
- Reset/latch: low pulse `>= 200 us`
- Supply voltage: `4.5 V` to `5.5 V`
- Input high threshold `VIH` minimum: `2.7 V`
- LED driving for this project should use `PIO`, not cycle-sensitive bit-banging.
- Keep the LED protocol implementation isolated in its own module, such as `led.c`, `led.h`, and a dedicated PIO program.

## Safety Rules

- Default all motor outputs to disarmed/off during startup, reset, network loss, and internal faults.
- Never assume a connected propeller-free bench setup.
- Clamp all command inputs.
- Add timeouts for controller heartbeat and communication loss.
- Prefer explicit state machines over implicit arming behavior.
- Keep hardware-specific constants named and centralized.

## Coding Guidelines

- Keep peripheral code split by function: IMU, ESC/DSHOT, battery ADC, telemetry, LED, networking.
- Avoid mixing control logic with transport/networking code.
- Document units in structs and variable names when possible, such as `voltage_v`, `gyro_dps`, or `accel_g`.
- When touching shared hardware resources like PIO, DMA, SPI, or ADC, document ownership clearly.

## Known Assumptions

These assumptions are currently based on project notes and existing code:

- The BMI270 uses SPI, not I2C.
- GPIO 18 current sense is likely analog input from the ESC.
- The referenced ESC documentation is the SpeedyBee F405 V4 BLS 55A 30x30 stack manual/product page.

If any of those are wrong, update this file before building more features on top of them.

## Open Items To Confirm Later

- Whether current sense on GPIO 18 needs ADC scaling and what the calibration constant is
- Battery chemistry and expected voltage range
- Required control loop rate and IMU filter strategy
- Final Wi-Fi operating mode: station, access point, or both

## Reference Documentation

- Official SpeedyBee download page: https://www.speedybee.com/f405-v4-55a-stack-download/
- Official SpeedyBee product/spec page: https://www.speedybee.com/speedybee-f405-v4-bls-55a-30x30-fc-esc-stack/
- Harvatek LED datasheet mirror/spec summary: https://www.alldatasheet.com/datasheet-pdf/pdf/1548532/HARVATEK/B3DK3BRG-05C000113U1930.html

## Agent Behavior In This Repo

When making changes in this repository:

- Read the relevant hardware module before editing behavior.
- Preserve pin assignments unless explicitly asked to remap hardware.
- Call out safety implications before changing motor-control behavior.
- Prefer incremental bring-up over large rewrites.
- Keep documentation in sync when adding new peripherals or changing pin use.
