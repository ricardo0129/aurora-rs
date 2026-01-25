//! This example toggles the GPIO25 pin, using a PIO program compiled via pio::pio_asm!().
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
#![no_std]
#![no_main]

use hal::clocks::{init_clocks_and_plls, Clock};
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::watchdog::Watchdog;
use hal::Sio;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::pio::Tx; // important! brings the write() method into scope

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
pub fn color_as_u32(red: u8, green: u8, blue: u8) -> u32 {
    let color: u32 = ((blue as u32) << 16) | ((red as u32) << 8) | (green as u32);
    color
}

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
    let led_pin_id = led.id().num;

    let (T1, T2, T3) = (4, 6, 2);
    // Define some simple PIO program.
    let program = pio_proc::pio_asm!(
        ".side_set 1",
        ".define public T1 4",
        ".define public T2 6",
        ".define public T3 2",
        ".wrap_target",
        "bitloop:",
        "   out x, 1        side 0 [T3 - 1]",
        "   jmp !x do_zero  side 1 [T1 - 1]",
        "do_one:",
        "   jmp bitloop     side 1 [T2 - 1]",
        "do_zero:",
        "   nop             side 0 [T2 - 1]",
        ".wrap",
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let sys_clk = clocks.system_clock.freq().to_Hz() as f32;
    let bit_freq = 800_000.0 * (T1 + T2 + T3) as f32;
    let div = sys_clk / bit_freq;

    let int = div as u16;
    let frac = ((div - int as f32) * 256.0) as u8;
    /*
        let (mut sm, _, mut tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(led_pin_id, 1)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);
    */
    let (mut sm, _, mut tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(24)
        .side_set_pin_base(led_pin_id)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    let red: u8 = 0;
    let green: u8 = 0;
    let blue: u8 = 255;

    // Pack into a 24-bit word, GRB order
    let color: u32 = ((blue as u32) << 16) | ((red as u32) << 8) | (green as u32);

    let colors: [(u8, u8, u8); 3] = [(255, 0, 0), (0, 255, 0), (0, 0, 255)];

    // PIO runs in background, independently from CPU
    let mut iter: i32 = 0;
    loop {
        let c = colors[(iter % 3) as usize];
        for i in 0..8 {
            tx.write(color_as_u32(c.0, c.1, c.2));
        }
        cortex_m::asm::delay(125_000_000); // ~80µs @ 125MHz
        iter += 1;
    }
}
