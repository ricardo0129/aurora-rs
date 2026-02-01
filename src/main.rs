#![no_std]
#![no_main]
mod communication;
mod key_codes;
mod keyboard;
mod layout;
mod sk6812;

use crate::communication::serial::{serial_read_byte, serial_write_byte};
use crate::keyboard::key_matrix::{scan_keys, Column, Row, StateMatrix};
use crate::keyboard::Side;
use crate::sk6812::{color_as_u32, LedController};

use cortex_m::delay::Delay;
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

use crate::key_codes::KeyCode::{self, *};
use hal::fugit::ExtU32;
//use hal::fugit::RateExtU32;
use hal::gpio::{
    DynPinId, FunctionPio0, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullNone, PullUp,
};
use hal::pio::PIOExt;
//use hal::uart::{DataBits, StopBits, UartConfig};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig,
    },
};
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rp2040_hal::entry]
fn main() -> ! {
    info!("initializing split");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
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
    let mut hid = HIDClass::new_with_settings(
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
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid).build();

    let side: Side = Side::LEFT;
    let key_mapping = match side {
        Side::LEFT => &layout::L_LAYER,
        Side::RIGHT => &layout::R_LAYER,
    };

    let (mut cols, mut rows, led_pin, mut data_pin) = initialize_pins(&side, pins);

    let led_pin_id: u8 = led_pin.id().num;
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let sys_clk = clocks.system_clock.freq().to_Hz() as f32;
    let mut leds: LedController<17> = LedController::new(&mut pio, sm0, led_pin_id, sys_clk);

    let mut scan_countdown = timer.count_down();
    scan_countdown.start(10u32.millis());
    let colors: [u32; 3] = [
        color_as_u32(0, 120, 0),
        color_as_u32(20, 30, 20),
        color_as_u32(127, 44, 12),
    ];
    for i in 0..17 {
        leds.set_pixel(i, colors[0]);
    }
    leds.show();
    loop {
        dev.poll(&mut [&mut hid]);

        if scan_countdown.wait().is_ok() {
            info!("scan keys");
            let state = scan_keys(&mut rows, &mut cols, &mut delay);
            debug!("build report");
            let report = build_report(&state, key_mapping);
            hid.push_input(&report).ok();
        }
        // drop received data
        hid.pull_raw_output(&mut [0; 64]).ok();
    }
}

pub fn initialize_pins(
    side: &Side,
    pins: gpio::bank0::Pins,
) -> (
    [Column; layout::COLS],
    [Row; layout::ROWS],
    Pin<hal::gpio::bank0::Gpio0, FunctionPio0, PullDown>,
    Pin<hal::gpio::bank0::Gpio1, FunctionSioOutput, PullDown>,
) {
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
            let led_pin: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
            let data_pin = pins.gpio1.into_push_pull_output();
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
            let led_pin: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
            let data_pin = pins.gpio1.into_push_pull_output();
            (cols, rows, led_pin, data_pin)
        }
    }
}

fn build_report(
    matrix: &StateMatrix,
    key_mapping: &[[KeyCode; layout::COLS]; layout::ROWS],
) -> KeyboardReport {
    let mut keycodes = [0u8; 6];
    let mut keycode_count = 0;
    let mut push_key = |keycode: u8| {
        keycodes[keycode_count] = keycode;
        keycode_count += 1;
    };
    let modifier = 0;
    for (row_idx, row) in matrix.iter().enumerate() {
        for (col_idx, &pressed) in row.iter().enumerate() {
            if pressed {
                push_key(key_mapping[row_idx][col_idx] as u8);
            }
        }
    }

    KeyboardReport {
        modifier,
        reserved: 0,
        leds: 0,
        keycodes,
    }
}
