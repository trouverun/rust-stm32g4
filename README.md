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
- **bootloader**: binary crate containing the bootloader

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

The real-time constraint at 40 kHz FOC rate is satisfied, with the full FOC ISR (estimation + FOC) executing on average in 14.72 µs (min: 14.59 µs, max: 14.98 µs, N=400), always well within the 25 µs budget.

Next, the closed-loop current control performance was evaluated using the branch "bandwidth-test", which includes a firmware-level routine for injecting sine wave torque setpoints, composed of the specified frequencies and the given amplitude. The derived q-axis current setpoint and the measured q-axis current is recorded to RAM at the full 40 kHz FOC rate, and retrieved with probe-rs.

The plot below shows the response to a 300 Hz sine wave setpoint, showing adequate tracking performance with some phase lag:

  <img width="1800" height="1050" alt="tracking" src="https://github.com/user-attachments/assets/ab43a091-d069-4650-b012-c66bb56e5263" />

For a more comprehensive test, a sum of sine waves at 14 odd harmonics of 100 Hz, spanning from 100 Hz to 3.9 kHz was fed as setpoint instead. The setpoints and the measured response were coherently averaged across 15 periods of excitation, and a Discrete Fourier Transform (DFT) was applied to them. The closed loop gain was then computed as the ratio of output spectrum to the setpoint spectrum at each excitation frequency, and the data points were interpolated to find the -3 dB crossing point, which gives the closed-loop bandwidth:

<img width="1800" height="1650" alt="bandwidth" src="https://github.com/user-attachments/assets/32ccd71a-9e6c-4431-be7c-722a0888878d" />

The estimated bandwidth value of 939 Hz lands near the specified tuning goal of 1 kHz, with some deviation caused by the unmodelled inverter nonlinearities (deadtime, capacitance).

</details>

<details>
  <summary><h2>CAN interface details</h2></summary>
  
  The CAN interface is defined and documented by the [DBC](/firmware/dbc/can.dbc) file:
  
  <img width="1385" height="678" alt="dbc" src="https://github.com/user-attachments/assets/da8f2a8f-8015-4a2f-8624-dbd1952540ff" />
</details>

<details>
  <summary><h2>Adding support for a new board</h2></summary>
  
  A new board needs to satisfy the following prerequisites hardware-wise to be compatible:
  - STM32G4 MCU
  - Low side 3-shunt sensing
  - "Dumb" gate driver, i.e. deadtime is handled by the MCU timer peripheral, not by the gate driver

  The firmware also makes the following assumptions about the hardware, deviating from these will require minor (10-20 LOC) changes to the firmware code:
  - Board temperature and DC bus voltage are sampled using the same ADC as phase currents (regular conversions pre-empted by injected conversions)

  ### Cargo.toml
  The first thing needed is a new board entry in [/bootloader/Cargo.toml](/bootloader/Cargo.toml), as shown below for the zest1:
  ```rust
  board-zest1 = ["embassy-stm32/stm32g473qe"]
  ```

  and in [/firmware/Cargo.toml](/firmware/Cargo.toml): 
  ```rust
  board-zest1 = [
    "mcu-opamps", "overcurrent-comparators",
    "embassy-stm32/stm32g473qe", "rtic-monotonics/stm32g473qe",
  ]
  ```
  The entry needs to follow the naming convention of "board-{yourboard}", and specify the supported features (e.g. MCU side opamps) and the embassy/RTIC flags for the chip (stm32g473qe in this case).

  ### New board file

  The next step is to create a new board file in /firmware/src/boards/, which populates the following structs, defined in [/firmware/src/boards/mod.rs](/firmware/src/boards/mod.rs), using the embassy peripherals actually used by the board:

  ```rust
  pub struct BoardInfo {
    pub current_limit_a: f32,
    pub dc_voltage_limit_v: f32,
    pub mosfet_deadtime_ns: u32,
    pub mosfet_on_delay_ns: u32,
    pub mosfet_off_delay_ns: u32,
    pub deadtime_compensation_band_a: f32
  }

  #[cfg(feature = "mcu-opamps")]
  pub struct ShuntOpAmps {
      u: OpAmpOutput<'static, OpAmpU>,
      v: OpAmpOutput<'static, OpAmpV>,
      w: OpAmpOutput<'static, OpAmpW>,
  }

  pub struct AdcFeedbackMappings {
      #[cfg(feature = "mcu-opamps")]
      pub opamps: ShuntOpAmps,
      pub adc_a: Peri<'static, FeedbackAdcA>,
      pub adc_b: Peri<'static, FeedbackAdcB>,
      pub u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
      pub v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
      pub w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
      pub vbus_channel: AnyAdcChannel<'static, FeedbackAdcA>,
      pub tboard_channel: AnyAdcChannel<'static, FeedbackAdcB>,
      pub sample_trigger: BasicTrgoOutput<'static, AdcFeedbackTimer>,
  }

  pub struct HallFeedbackMappings {
      pub hall_timer: HallSensor<'static, HallFeedbackTimer>,
  }

  pub struct SPIMappings {
      pub spi: Spi<'static, Blocking, Master>,
      pub cs: Output<'static>
  }

  #[cfg(feature = "overcurrent-comparators")]
  pub struct CurrentComparators {
      pub dac_dual: Dac<'static, ComparatorDacDual, Blocking>,
      pub comp_u: Comp<'static, CompU>,
      pub comp_v: Comp<'static, CompV>,
      pub comp_w: Comp<'static, CompW>,
  }

  pub struct PwmOutputMappings {
      #[cfg(feature = "overcurrent-comparators")]
      pub comparators: CurrentComparators,
      pub pwm: PWM<'static, PwmTimer, NotRunning>,
      pub deadtime: PwmDeadtime,
  }
  ```

  The board file also needs to implement the following measurement abstractions:
  ```rust
  /// Phase current from the shunt opamp output voltage
  pub fn measurement_v_to_a(v: f32) -> f32 {}

  /// Opamp output voltage at the given phase current magnitude
  pub fn limit_a_to_v(current_limit_a: f32) -> f32 {}

  /// Board temperature from the thermistor measurement voltage
  pub fn v_to_c(v: f32) -> f32 {}

  /// DC bus voltage from the divider measurement voltage
  pub fn vbus_measurement_v_to_v(v: f32) -> f32 {}
  ```

  To enable [/firmware/src/main.rs](/firmware/src/main.rs) to generate interrupt handlers for the correct peripherals, the board file needs to export the interrupt handler names, like so:

  ```rust
  #[macro_export]
  macro_rules! board_irqs {
      ($cb:ident) => {
          $cb!(
              foc = ADC3,
              pwm_break = TIM8_BRK,
              hall = TIM3,
              watchdog = TIM7_DAC,
              can = FDCAN1_IT0,
              dispatchers = [SPI2, SPI3, UART5]
          );
      };
  }
  ```

  The file [/firmware/src/boards/zest1.rs](/firmware/src/boards/zest1.rs) can be used as a reference for the above steps.

  To register the board file, it needs to be included from [/firmware/src/boards/mod.rs](/firmware/src/boards/mod.rs) behind the feature flag created in the previous step:
  ```rust
  #[cfg(feature = "board-zest1")]
  mod zest1;
  #[cfg(feature = "board-zest1")]
  pub use zest1::*;
  ```
  
  ### Linker memory layout
  The compiler needs to know the memory layout of the device, which should be specified using new files in two places:
  - /bootloader/memory/
  - /firmware/memory/
  
  Examples are shown below for the zest1 board:
  ```rust
  // bootloader/memory/zest1.x
  MEMORY
  {
      FLASH            : ORIGIN = 0x08000000, LENGTH =  16K
      RAM              : ORIGIN = 0x20000000, LENGTH =  96K
  }

  // firmware/memory/zest1.x
  MEMORY
  {
      FLASH  : ORIGIN = 0x08004000, LENGTH = 236K /* BANK_1 (offset by 16K bootloader) */
      RAM    : ORIGIN = 0x20000000, LENGTH =  96K /* SRAM1 + SRAM2 */
      CCMRAM : ORIGIN = 0x10000000, LENGTH =  32K /* zero wait state code */
  }
  ```
  The exact values used should respect the chip specific limits found on the datasheet. **The file names need to match the feature flag** (e.g. board-example -> example.x).

  ### Build and flash the binaries
  Now all that is left to do is build the binaries, as shown below for the zest1 board:
  ```bash
  cd bootloader
  cargo build --release --features board-zest1
  cd ../firmware
  cargo build --release --features board-zest1
  ```
  Correspondingly, the binaries can be flashed via STLink, with the bootloader flashed first:
  ```bash
  cd bootloader
  cargo flash --release --features board-zest1 --chip STM32G473QETx
  cd ../firmware
  cargo run --release --features board-zest1 -- --chip STM32G473QETx
  ```
  At this stage further firmware flashing can be done through CAN.
</details>
