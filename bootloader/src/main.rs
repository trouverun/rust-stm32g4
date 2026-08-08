#![no_std]
#![no_main]

mod loader;
pub mod pac {
    pub use embassy_stm32::pac::*;
}

use cortex_m_rt::{entry, exception};
use embassy_stm32::{
    Config, flash::{Flash}, 
    wdg::IndependentWatchdog
};
use firmware_core::{BootloaderState, DecodeResult, SwapMode};
use loader::*;

#[entry]
fn main() -> ! {
    let ep = embassy_stm32::init(Config::default());
    let flash = Flash::new_blocking(ep.FLASH);
    let mut bootloader = Bootloader::new(flash);

    let bootloader_status = bootloader.read_status();
    match bootloader_status {
        DecodeResult::Corrupt => bootloader.write_status(BootloaderState::DfuContentsRejected),
        DecodeResult::Valid(status) => {
            match status.state {
                BootloaderState::DfuFreshlyWritten => {
                    bootloader.perform_swap(SwapMode::Normal);
                    bootloader.write_status(BootloaderState::SwappedImageTrialBooted);
                },
                BootloaderState::SwappedImageTrialBooted => {
                    let watchdog_reboot = embassy_stm32::pac::RCC.csr().read().iwdgrstf();
                    if watchdog_reboot {
                        bootloader.perform_swap(SwapMode::Revert);
                        bootloader.write_status(BootloaderState::SwappedImageBootTimeout);
                    }
                },
                _ => {}
            }
        },
        DecodeResult::Empty => {}
    }

    let mut watchdog = IndependentWatchdog::new(ep.IWDG, 10_000_000);
    watchdog.unleash();

    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.SCB.vtor.write(FIRMWARE_ORIGIN);
        cortex_m::asm::bootload(FIRMWARE_ORIGIN as *const u32)
    }
}

#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    cortex_m::asm::udf();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}
