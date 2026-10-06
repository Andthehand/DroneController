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

### LSM6DSV32X IMU

The active LSM6DSV32X driver uses `spi1` with this pinout:

- GPIO 12: SPI1 RX / MISO
- GPIO 11: SPI1 TX / MOSI
- GPIO 10: SPI1 SCK
- GPIO 13: IMU chip select
- GPIO 14: INT2 input, currently not used for sampling

### IMU Filtering

- Both sensors run at a nominal 1.92 kHz. The controller polls STATUS_REG (0x1E), requiring both accelerometer and gyro data-ready bits before reading a sample.
- Data-ready status is polled rather than waiting a fixed 1 ms. `CONTROL_LOOP_INTERVAL_US` does not govern this path; actual throughput needs verification on hardware.
- All three gyro and accelerometer axes pass through second-order Butterworth low-pass filters before the Kalman attitude estimator. Raw integer samples remain unchanged.
- `IMU_GYRO_LPF_HZ` defaults to 100 Hz and `IMU_ACCEL_LPF_HZ` to 30 Hz in `src/config.h`. Setting either cutoff to zero bypasses that software filter. The existing internal gyro filter remains enabled.
- Coefficients use the measured interval between fresh reads to accommodate polling jitter and missed samples. The first sample seeds filter history; startup/recovery uses the nominal sample interval.
- `IMU_POLL_INTERVAL_US` is 50 us while waiting for data. `IMU_SAMPLE_TIMEOUT_US` is 10000 us. Read failures or stale samples disarm outputs, clear the arm request, and reset filter/PID state. Recovery requires a new explicit arm request.
- These are bench starting settings, not flight-certified tuning. Lower cutoffs add control delay; filtering cannot repair clipping, aliased noise, or mechanical imbalance.

Bench verification:

1. Remove propellers before powering motors. Check for loose mounting and damaged motors/props; use suitable controller vibration isolation.
2. Confirm startup tilt is stable and manually tilting the board still produces prompt, correctly signed roll/pitch changes.
3. With motors off, establish a tilt-noise baseline. Compare with motors running at several low throttle settings and check the reported PID rate while control is connected. Fresh-sample processing should not exceed the nominal 1920 Hz sensor rate on average.
4. Verify sensor-failure disarming on a controlled bench before flight. Restore samples, confirm motors stay disarmed, and explicitly re-arm at zero throttle.
5. If vibration remains, capture raw sensor data and a frequency spectrum before choosing notch frequencies or changing cutoffs. Props-off testing does not reproduce propeller-loaded vibration; PID stability and real sample timing remain hardware validation requirements.

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

- The active LSM6DSV32X uses SPI, not I2C.
- GPIO 18 current sense is likely analog input from the ESC.
- The referenced ESC documentation is the SpeedyBee F405 V4 BLS 55A 30x30 stack manual/product page.

If any of those are wrong, update this file before building more features on top of them.

## Open Items To Confirm Later

- Whether current sense on GPIO 18 needs ADC scaling and what the calibration constant is
- Battery chemistry and expected voltage range
- Sustained fresh-sample loop rate and IMU filter tuning under real motor vibration
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
