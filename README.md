# Rust-STM32G4
### Firmware for STM32G4 based motor control boards.

[![CI](https://github.com/trouverun/rust-stm32g4/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/trouverun/rust-stm32g4/actions/workflows/ci.yml)

## Features
- Torque control with Field Oriented Control (FOC)
- Field weakening control
- Full speed range sensorless rotor angle estimation
- Automatic motor parameter identification
- Current control PI autotuning
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
  <summary><h2>Evaluation on the STM32 ZEST1S discovery kit</h2></summary>
The firmware was tested on the setup shown below:
  
  <img width="4032" height="3024" alt="test_setup" src="https://github.com/user-attachments/assets/061007d8-c08d-431e-ad24-0f85361c4255" />

[B-G473E-ZEST1S](https://www.st.com/en/evaluation-tools/b-g473e-zest1s.html#overview)
[STEVAL-LVLP01](https://www.st.com/en/evaluation-tools/steval-lvlp01.html)
[B-MOTOR-PMSMA1](https://www.st.com/en/evaluation-tools/b-motor-pmsma1.html)

The firmware configuration used was as follows:
- 40 kHz FOC rate (matching the PWM switching rate, i.e. single update PWM)
- Motor parameters were identified using the (fully sensorless) self-commissioning routine built to the firmware
- Current loop PI controllers were autotuned with a closed-loop bandwidth tuning goal of 1 kHz
- Rotor angle from sensorless feedback (rotor angle from a digital Hall sensor was recorded for comparisons only)

### Current control loop bandwidth
The current control loop performance was evaluated using firwmare built with the cargo feature `bandwidth-test`, which includes a routine for injecting high frequency sine wave torque setpoints to the FOC loop, and recording the derived d,q-axis current setpoints and the measured currents into RAM at the full 40 kHz FOC rate.

The response to a 300 Hz sine wave torque setpoint shows satisfactory q-axis current tracking and d-axis current regulation performance:

<img width="1000" height="900" alt="image" src="https://github.com/user-attachments/assets/07c7bc02-41fa-4ff7-a404-5f972120dc87" />

To properly evaluate the current control bandwidth, a sum of sine waves (multisine) was fed as the torque setpoint instead. The multisine consisting of 14 odd harmonics of 100 Hz (spanning from 100 Hz to 2.7 kHz) was applied for a total of 7 periods (7*10 ms). The data was post-processed using `scripts/analysis.py` which first coherently averages the 7 periods of setpoint->output data before applying a discrete fourier transform. The closed loop gain was then computed as the ratio of output spectrum to the setpoint spectrum at each of the 14 excitation frequencies, and the data points were interpolated to find the -3 dB crossing point, which gives the closed-loop bandwidth:

<img width="1800" height="1650" alt="bandwidth" src="https://github.com/user-attachments/assets/080d16fc-7de7-4555-a71d-76e5c0f35a3a" />

The estimated current control loop bandwidth of 921 Hz deviates from the targeted bandwidth of 1 kHz by around 8 %. The target bandwidth is specified for an ideal second order system so some real-world deviation is to be expected.

### Sensorless rotor angle estimation
The performance of the sensorless rotor angle estimation was evaluated using the following test sequence:

1. Torque was commanded by a PID loop on the host PC to regulate the mechanical rotor speed to around 5 rad/s
2. The velocity setpoint was tapered down to bring the rotor to a stop
3. Torque was commanded by a PID loop on the host PC to regulate the mechanical rotor speed to around -5 rad/s
4. The velocity setpoint was tapered down to bring the rotor to a stop
5. Constant torque command was held for 3 seconds

The rotor angle interpolated from digital Hall sensor readings was not used for control, but is shown as a rough ground truth reference. The below plot visualizes the experiment using data collected via CAN:

<img width="1800" height="1350" alt="image" src="https://github.com/user-attachments/assets/28054062-9f8c-42e0-9565-8276a36d0c15" />

The sensorless estimator matches the Hall interpolated rotor angle even at very low velocities. Note that the Hall interpolated angle itself can be off by up to 1 rad on the 4 pole motor used in testing, which shows up as additional error during standstill and acceleration. The clearly visible D-axis current ripple at low angular velocities is caused by the high frequency voltage injection used in the saliency observer.

### Execution rate and jitter
The execution time and jitter charasteristics of the real-time FOC loop was analyzed by toggling GPIO pins at various points of the FOC ISR during torque control. The operating envelope spanned from standstill up to field weakening region to cover all execution paths. The GPIO outputs were captured using a logic analyser as shown below:

<img width="1451" height="483" alt="image" src="https://github.com/user-attachments/assets/272466a9-3706-41dc-aa80-bb0cb3596df8" />

The statistics computed from 400k ISR executions (i.e. 10s worth of data) are summarized in the below table:

| Metric | Budget (40 kHz FOC) | Avg | Min | Max |
|---|---|---|---|---|
| Latency from ISR entry to duty cycles written to timer register | 12.5 µs | 9.10 µs | 9.04 µs | 9.14 µs |
| Full ISR execution time | 25 µs | 17.60 µs | 17.53 µs | 17.71 µs |
| Delay of ISR entry from nominal schedule | - | - | -| 0.73 µs |

At 40 kHz FOC rate around 71 % of the available CPU cycles are used (17.71 µs / 25 µs), which still leaves sufficient CPU cycles for CAN interfacing. 

The main constraint for the FOC rate is the time available to compute new duty cycles. The phase currents are sampled at the PWM midpoint, and to minimize delay in the current control loop, the new duty cycles must be written before the next PWM period starts. The timing is quite tight at 40 kHz:

|  | Value |
|---|---|
| Time budget (time from current sampling to start of next PWM period) | 12.5 µs |
| Max latency from ISR firing to duty cycles written | -9.14 µs |
| Max delay of ISR entry | -0.73 µs |
| **Worst-case margin** | **2.63 µs** |

The current measurement ADC conversion starts at the PWM midpoint and triggers the FOC ISR only once it completes, so the conversion time further reduces the effective margin.

</details>

<details>
  <summary><h2>CAN interface details</h2></summary>
  
  The CAN interface is defined and documented by the [DBC](/firmware/dbc/can.dbc) file:
  <img width="2077" height="1069" alt="image" src="https://github.com/user-attachments/assets/ec53ca55-423b-4e78-af28-087141005ed8" />
  
</details>

<details>
  <summary><h2>Adding support for a new board</h2></summary>
  
  A new board needs to satisfy the following prerequisites hardware-wise to be compatible:
  - STM32G4 MCU with dual-bank flash
  - Low side 3-shunt sensing through two ADCs (sharing one of the phases)
  - "Dumb" gate driver, i.e. deadtime is handled by the MCU timer peripheral, not by the gate driver

  The firmware also makes the following assumptions about the hardware, deviating from these will require minor (10-20 LOC) additional changes to the firmware code:
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
  The entry needs to follow the naming convention of "board-{yourboard}", and specify the supported features (e.g. MCU side opamps, SPI encoders, etc.) along with the embassy/RTIC flags for the chip (stm32g473qe in this case).

  ### New board file

  The next step is to create a new board file in [/firmware/src/boards/](/firmware/src/boards/), which implements the `Board` trait defined in [/firmware/src/boards/mod.rs](/firmware/src/boards/mod.rs):

  ```rust
  pub trait Board {
      #[cfg(feature = "mcu-opamps")]
      type OpAmpU: opamp::Instance;
      #[cfg(feature = "mcu-opamps")]
      type OpAmpV: opamp::Instance;
      #[cfg(feature = "mcu-opamps")]
      type OpAmpW: opamp::Instance;
      type FeedbackAdcA: adc::Instance + HasInjectedTrigger + HasRegularTrigger;
      type FeedbackAdcB: adc::Instance + HasInjectedTrigger + HasRegularTrigger;
      type AdcFeedbackTimer: BasicInstance;
      type HallFeedbackTimer: GeneralInstance4Channel;
      #[cfg(feature = "overcurrent-comparators")]
      type CompU: comp::Instance;
      #[cfg(feature = "overcurrent-comparators")]
      type CompV: comp::Instance;
      #[cfg(feature = "overcurrent-comparators")]
      type CompW: comp::Instance;
      #[cfg(feature = "overcurrent-comparators")]
      type ComparatorDacDual: dac::Instance;
      type PwmTimer: AdvancedInstance4Channel;
      type SoftWatchdogTimer: BasicInstance;
  
      const FEEDBACK_TRIGGER_A: <Self::FeedbackAdcA as HasInjectedTrigger>::Trigger;
      const FEEDBACK_TRIGGER_B: <Self::FeedbackAdcB as HasInjectedTrigger>::Trigger;
      const BOARD_FEEDBACK_TRIGGER: <Self::FeedbackAdc as HasRegularTrigger>::Trigger;
      const INFO: BoardInfo;
  
      /// Phase current from the shunt opamp output counts
      fn current_adc_to_a(counts: i16) -> f32;
      /// Opamp output voltage at the given phase current magnitude
      #[cfg(feature = "overcurrent-comparators")]
      fn limit_a_to_v(current_limit_a: f32) -> f32;
      /// Board temperature from the thermistor measurement counts
      fn temperature_adc_to_c(counts: u16) -> f32;
      /// DC bus voltage from the divider measurement counts
      fn vbus_adc_to_v(counts: u16) -> f32;
  
      fn map_peripherals() -> PeripheralMappings;
  }
  ```
  Where the mappings structs returned by `map_peripherals()` carry the embassy peripheral selections:
  ```rust
  pub struct PeripheralMappings {
    pub current_feedback: AdcFeedbackMappings,
    #[cfg(feature = "hall-feedback")]
    pub hall_feedback: HallFeedbackMappings,
    pub pwm_output: PwmOutputMappings,
    pub acceleration: AccelerationMappings,
    pub memory: MemoryMappings,
    pub can: CanMappings,
    pub watchdog: WatchdogMappings,
    pub debug: DebugMappings,
  }

  #[cfg(feature = "mcu-opamps")]
  pub type OpAmpU = <Active as Board>::OpAmpU;
  #[cfg(feature = "mcu-opamps")]
  pub type OpAmpV = <Active as Board>::OpAmpV;
  #[cfg(feature = "mcu-opamps")]
  pub type OpAmpW = <Active as Board>::OpAmpW;
  pub type FeedbackAdcA = <Active as Board>::FeedbackAdcA;
  pub type FeedbackAdcB = <Active as Board>::FeedbackAdcB;
  pub type AdcFeedbackTimer = <Active as Board>::AdcFeedbackTimer;
  // Other type aliases...
  
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
      pub phase_sample_time: SampleTime,
      pub vbus_sample_time: SampleTime,
      pub tboard_sample_time: SampleTime,
  }

  // Other mapping structs...
  ```

  Besides implementing the `Board` trait, the board file needs to define a macro which exports valid names for use in RTIC task interrupt binding, where the names match the peripherals actually used by the board:

  ```rust
  #[macro_export]
  macro_rules! board_irqs {
      ($cb:ident) => {
          $cb!(
              foc_isr = ADCx,
              pwm_break_isr = TIMx_BRK,
              hall_isr = TIMx,
              soft_watchdog_isr = TIMx_DAC,
              can_isr = FDCANx_IT0,
              dispatchers = [SPIx, SPIx, UARTx]
          );
      };
  }
  ```

  The file [/firmware/src/boards/zest1.rs](/firmware/src/boards/zest1.rs) should be used as a reference for the above steps.

  To register the new board file, it needs to be included from [/firmware/src/boards/mod.rs](/firmware/src/boards/mod.rs) behind the feature flag created in the first step, and type defined as the active board:
  ```rust
  #[cfg(feature = "board-zest1")]
  mod zest1;
  #[cfg(feature = "board-zest1")]
  pub type Active = zest1::Zest1;
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
      FLASH : ORIGIN = 0x08000000, LENGTH =  16K
      RAM   : ORIGIN = 0x20000000, LENGTH =  96K
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
