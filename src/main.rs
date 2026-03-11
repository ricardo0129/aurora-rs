#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use nrf52840_hal as _;
use panic_probe as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    info!("Hello world");

    loop {}
}
