#![no_std]
#![no_main]
mod communication;
mod key_codes;
mod keyboard;
mod layout;
mod sk6812;

use crate::keyboard::key_matrix::{Column, OutPin, PioPin, Row};
use crate::keyboard::Keyboard;
use crate::layout::Side;
use crate::sk6812::{color_as_u32, LedController};

use cortex_m::prelude::*;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our hal so we can switch targets quickly.
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
};
use usb_device as usbd;
use usbd::{
    class_prelude::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};

use hal::fugit::ExtU32;
use hal::gpio::{FunctionPio0, Pin, PullDown};
use hal::pio::PIOExt;
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig,
    },
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000;
const SCAN_LOOP_INTERVAL_MS: u32 = 10;
const LED_COUNT: usize = 17;

#[rp2040_hal::entry]
fn main() -> ! {
    info!("initializing split");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        EXTERNAL_XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let bus_allocator = UsbBusAllocator::new(bus);
    let vid_pid = UsbVidPid(0x6666, 0x0789);
    let hid = HIDClass::new_with_settings(
        &bus_allocator,
        KeyboardReport::desc(),
        10,
        HidClassSettings {
            subclass: HidSubClass::NoSubClass,
            protocol: HidProtocol::Keyboard,
            config: ProtocolModeConfig::ForceReport,
            locale: HidCountryCode::NotSupported,
        },
    );
    let dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid).build();

    let side: Side = Side::LEFT;

    let (cols, rows, led_pin, data_pin) = initialize_pins(&side, pins);
    let keyboard_delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut scan_countdown = timer.count_down();
    scan_countdown.start(SCAN_LOOP_INTERVAL_MS.millis());
    let mut keyboard = Keyboard::new(
        side,
        rows,
        cols,
        keyboard_delay,
        dev,
        hid,
        scan_countdown,
        data_pin,
    );

    let led_pin_id: u8 = led_pin.id().num;
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let sys_clk = clocks.system_clock.freq().to_Hz() as f32;
    let mut leds: LedController<LED_COUNT> = LedController::new(&mut pio, sm0, led_pin_id, sys_clk);

    let colors: [u32; 3] = [
        color_as_u32(60, 32, 0),
        color_as_u32(20, 30, 20),
        color_as_u32(127, 44, 12),
    ];
    for i in 0..17 {
        leds.set_pixel(i, colors[0]);
    }
    leds.show();
    loop {
        keyboard.tick();
    }
}

pub fn initialize_pins(
    side: &Side,
    pins: gpio::bank0::Pins,
) -> ([Column; layout::COLS], [Row; layout::ROWS], PioPin, OutPin) {
    match side {
        //LEFT SIDE
        //rows: 27 28 26 22
        //cols: 21 4 5 6 7
        Side::LEFT => {
            let cols: [Column; layout::COLS] = [
                pins.gpio21.into_pull_down_input().into_dyn_pin(),
                pins.gpio4.into_pull_down_input().into_dyn_pin(),
                pins.gpio5.into_pull_down_input().into_dyn_pin(),
                pins.gpio6.into_pull_down_input().into_dyn_pin(),
                pins.gpio7.into_pull_down_input().into_dyn_pin(),
            ];

            let rows: [Row; layout::ROWS] = [
                pins.gpio27
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
                pins.gpio28
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
                pins.gpio26
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
            ];
            let led_pin: Pin<_, FunctionPio0, PullDown> = pins.gpio0.into_function().into_dyn_pin();
            let data_pin = pins.gpio1.into_push_pull_output().into_dyn_pin();
            (cols, rows, led_pin, data_pin)
        }
        //RIGHT SIDE
        //rows: 22 26 27 20
        //cols: 9 8 7 6 5
        Side::RIGHT => {
            let cols: [Column; layout::COLS] = [
                pins.gpio9.into_pull_down_input().into_dyn_pin(),
                pins.gpio8.into_pull_down_input().into_dyn_pin(),
                pins.gpio7.into_pull_down_input().into_dyn_pin(),
                pins.gpio6.into_pull_down_input().into_dyn_pin(),
                pins.gpio5.into_pull_down_input().into_dyn_pin(),
            ];
            let rows: [Row; layout::ROWS] = [
                pins.gpio22
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
                pins.gpio26
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
                pins.gpio27
                    .into_push_pull_output()
                    .into_pull_type()
                    .into_dyn_pin(),
            ];
            let led_pin: Pin<_, FunctionPio0, PullDown> = pins.gpio0.into_function().into_dyn_pin();
            let data_pin = pins.gpio1.into_push_pull_output().into_dyn_pin();
            (cols, rows, led_pin, data_pin)
        }
    }
}
