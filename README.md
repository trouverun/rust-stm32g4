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
- Firmware update via CAN

## Repository details
The firmware is 100% Rust which uses [RTIC](https://github.com/rtic-rs/rtic) for scheduling in combination with the [Embassy](https://github.com/embassy-rs/embassy) STM32 Hardware Abstraction Library (HAL).

The repository is structured as follows:
- **embassy**: fork of [Embassy](https://github.com/embassy-rs/embassy) with new peripheral drivers required for motor control applications along with modifications of existing drivers tailored for use with RTIC scheduling
- **field-oriented**: testable library crate with math and algorithmic code for motor control and estimation
- **firmware-core**: testable library crate with application-level logic
- **firmware**: binary crate containing the main hardware-dependent firmware to be flashed on the target 

<details open>
  <summary><h2>Demo running on the STM32 ZEST1S discovery kit</h2></summary>
The firmware was tested on the setup shown below:
  
  <img width="4032" height="3024" alt="test_setup" src="https://github.com/user-attachments/assets/061007d8-c08d-431e-ad24-0f85361c4255" />

[B-G473E-ZEST1S](https://www.st.com/en/evaluation-tools/b-g473e-zest1s.html#overview)
[STEVAL-LVLP01](https://www.st.com/en/evaluation-tools/steval-lvlp01.html)
[B-MOTOR-PMSMA1](https://www.st.com/en/evaluation-tools/b-motor-pmsma1.html)

The firmware configuration used was as follows:
- 40 kHz FOC rate
- Motor parameters were identified using the self-commissioning routine built to the firmware
- Current loop PI controllers were autotuned with a closed-loop bandwidth tuning goal of 1 kHz
- Hall feedback below 100 rad/s (electrical), sensorless above

First, during operation in torque control mode, GPIO pins were toggled from the FOC ISR to measure the execution time using a logic analyser:

<img width="1452" height="491" alt="foc_rate" src="https://github.com/user-attachments/assets/39abdfac-19c1-4892-8fd6-923dde3cc4c5" />

The real-time constraint at 40 kHz is satisfied, with the full ISR executing in 14.23 µs (well within the 25 µs budget).  

Next, the closed-loop current control performance was evaluated using the branch "bandwidth-test", which includes a firmware-level routine for injecting sine wave torque setpoints, composed of the specified frequencies and the given amplitude. The derived q-axis current setpoint and the measured q-axis current is recorded to RAM at the full 40 kHz FOC rate, and retrieved with probe-rs.

The plot below shows the response to a 300 Hz sine wave setpoint, showing adequate tracking performance with some phase lag:

  <img width="1800" height="1050" alt="tracking" src="https://github.com/user-attachments/assets/ab43a091-d069-4650-b012-c66bb56e5263" />

For a more comprehensive test, a sum of sine waves at 14 odd harmonics of 100 Hz, spanning from 100 Hz to 3.9 kHz was fed as setpoint instead. The setpoints and the measured response were coherently averaged across 15 periods of excitation, and a Discrete Fourier Transform (DFT) was applied to them. The closed loop gain was then computed as the ratio of output spectrum to the setpoint spectrum at each excitation frequency, and the data points were interpolated to find the -3 dB crossing point, which gives the closed-loop bandwidth:

<img width="1800" height="1650" alt="bandwidth" src="https://github.com/user-attachments/assets/32ccd71a-9e6c-4431-be7c-722a0888878d" />

The estimated bandwidth value of 939 Hz lands near the specified tuning goal of 1 kHz, with some deviation caused by the inverter nonlinearities (deadtime, capacitance) and any back-EMF caused by rotor movement due to q-axis current excitation.

</details>

<details>
  <summary><h2>CAN interface details</h2></summary>
  
  The CAN interface is defined and documented by the [DBC](/firmware/dbc/can.dbc) file:
  
  <img width="1385" height="678" alt="dbc" src="https://github.com/user-attachments/assets/da8f2a8f-8015-4a2f-8624-dbd1952540ff" />
</details>

<details>
  <summary><h2>Adding support for a new board</h2></summary>
  
  Prerequisites for the hardware:
  - Low side 3-shunt sensing
</details>
