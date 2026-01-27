//! This example toggles the GPIO25 pin, using a PIO program compiled via pio::pio_asm!().
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
#![no_std]
#![no_main]

mod sk6812;

use crate::sk6812::{color_as_u32, LedController};

use hal::clocks::{init_clocks_and_plls, Clock};
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::watchdog::Watchdog;
use hal::Sio;
use panic_halt as _;
use rp2040_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    // --- External crystal frequency ---
    // Pico & most RP2040 boards use 12 MHz
    const XTAL_FREQ_HZ: u32 = 12_000_000;

    // --- Initialize clocks & PLLs ---
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // configure LED pin for Pio0.
    let led: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
    // PIN id for use inside of PIO
    let led_pin_id: u8 = led.id().num;
    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let sys_clk = clocks.system_clock.freq().to_Hz() as f32;

    let mut leds: LedController<17> = LedController::new(&mut pio, sm0, led_pin_id, sys_clk);

    const N: usize = 4;
    let colors: [(u8, u8, u8); N] = [(0, 0, 120), (0, 120, 0), (100, 0, 0), (0, 100, 0)];

    // PIO runs in background, independently from CPU
    let mut iter: usize = 0;
    leds.show();
    loop {
        let c = colors[iter % N];
        for i in 0..17 {
            leds.set_pixel(i, color_as_u32(c.0, c.1, c.2));
        }
        leds.show();
        cortex_m::asm::delay(12_000_000); // ~80µs @ 125MHz
        iter += 1;
    }
}
