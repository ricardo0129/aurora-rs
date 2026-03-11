#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use nrf52840_hal as hal;
use nrf52840_hal::gpio::Level;
use panic_probe as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    info!("Hello, world!");
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let mut led = port0.p0_17.into_push_pull_output(Level::Low);
    loop {
        led.set_high().unwrap();
        defmt::println!("LED on");
        cortex_m::asm::delay(10_000_000);
        led.set_low().unwrap();
        defmt::println!("LED off");
        cortex_m::asm::delay(10_000_000);
    }
}
