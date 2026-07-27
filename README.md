# Rust-STM32G4
### Flexible firmware for STM32G4 based motor control boards.

[![CI](https://github.com/trouverun/rust-stm32g4/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/trouverun/rust-stm32g4/actions/workflows/ci.yml)

## Features
- Torque control with Field Oriented Control (FOC)
- Field weakening control
- Sensorless rotor angle/velocity estimation
- Automatic motor parameter identification
- Current control PI autotuning
- Hall sensor and SPI/RS485 encoder support
- Fault diagnostics and fault handling with per-fault reactions
- CAN interface

## Repository details
The firmware is 100% Rust which uses [RTIC](https://github.com/rtic-rs/rtic) for scheduling in combination with the [Embassy](https://github.com/embassy-rs/embassy) STM32 Hardware Abstraction Library (HAL).

The repository is structured as follows:
- **embassy**: fork of [Embassy](https://github.com/embassy-rs/embassy) with new peripheral drivers required for motor control applications along with modifications of existing drivers tailored for use with RTIC scheduling
- **field-oriented**: testable library crate with math and algorithmic code for motor control and estimation
- **firmware-core**: testable library crate with application-level logic
- **firmware**: binary crate containing the main hardware-dependent firmware to be flashed on the target 

<details>
<summary><h2>Demo running on the STM32 ZEST1S discovery kit</h2></summary>
</details>

<details>
<summary><h2>CAN interface details</h2></summary>
</details
>
<details>
<summary><h2>Adding support for a new board</h2></summary>
</details>