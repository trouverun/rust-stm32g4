use embassy_stm32::{Peri};
use embassy_stm32::spi::{Config as SpiConfig, Spi, Instance, MODE_0, MisoPin, MosiPin, SckPin};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::{Pin, Level, Output, Speed};

use crate::boards::SPIMappings;

pub fn spi_mappings<EncSpi, SpiSck, SpiMosi, SpiMiso, CsPin>(
    spi: Peri<'static, EncSpi>,
    sck: Peri<'static, SpiSck>,
    mosi: Peri<'static, SpiMosi>,
    miso: Peri<'static, SpiMiso>,
    cs: Peri<'static, CsPin>
) -> SPIMappings
    where
    EncSpi: Instance,
    SpiSck: SckPin<EncSpi>,
    SpiMosi: MosiPin<EncSpi>,
    SpiMiso: MisoPin<EncSpi>,
    CsPin: Pin,
{
    let mut spi_config = SpiConfig::default();
    spi_config.mode = MODE_0;
    spi_config.frequency = Hertz(2_000_000);
    spi_config.gpio_speed = Speed::VeryHigh;
    let spi =  Spi::new_blocking(spi, sck, mosi, miso, spi_config);

    SPIMappings {
        spi,
        cs: Output::new(cs, Level::High, Speed::VeryHigh)
    }
}