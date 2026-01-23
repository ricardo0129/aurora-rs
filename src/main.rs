#![no_std]
#![no_main]
mod communication;
mod key_codes;
mod keyboard;

use crate::keyboard::key_matrix;

use core::convert::Infallible;
use cortex_m::delay::Delay;
use cortex_m::{asm, prelude::*};

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our hal so we can switch targets quickly.
use rp2040_hal as hal;

use embedded_hal::digital::{InputPin, OutputPin};
use hal::fugit::ExtU32;
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
use hal::gpio::{DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullNone, PullUp};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig,
    },
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const ROWS: usize = 3;
const COLS: usize = 5;
const CYCLE_DELAY: u32 = 100;

#[rp2040_hal::entry]
fn main() -> ! {
    let res = key_matrix::hello();
    info!("{}", res);
    info!("Program start");
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
    /*
        let cols = match side {
            Side::LEFT => [
                pins.gpio9.into_pull_down_input(),
                pins.gpio8.into_pull_down_input(),
                pins.gpio7.into_pull_down_input(),
                pins.gpio6.into_pull_down_input(),
                pins.gpio5.into_pull_down_input(),
            ],
            Side::RIGHT => [
                pins.gpio21.into_pull_down_input(),
                pins.gpio4.into_pull_down_input(),
                pins.gpio5.into_pull_down_input(),
                pins.gpio6.into_pull_down_input(),
                pins.gpio7.into_pull_down_input(),
            ],
        };
    */
    /*
     */
    let (mut cols, mut rows) = initialize_split(side, pins);

    let mut scan_countdown = timer.count_down();
    scan_countdown.start(10u32.millis());

    loop {
        dev.poll(&mut [&mut hid]);

        if scan_countdown.wait().is_ok() {
            info!("scan keys");
            let state = scan_keys(&mut rows, &mut cols, &mut delay);
            debug!("build report");
            let report = build_report(&state);
            hid.push_input(&report).ok();
        }
        // drop received data
        hid.pull_raw_output(&mut [0; 64]).ok();
    }
}
pub enum Side {
    LEFT,
    RIGHT,
}

pub fn initialize_split(side: Side, pins: gpio::bank0::Pins) -> ([Column; COLS], [Row; ROWS]) {
    match side {
        //LEFT SIDE
        //rows: 27 28 26 22
        //cols: 21 4 5 6 7
        Side::LEFT => {
            let cols: [Column; COLS] = [
                pins.gpio21.into_pull_down_input().into_dyn_pin(),
                pins.gpio4.into_pull_down_input().into_dyn_pin(),
                pins.gpio5.into_pull_down_input().into_dyn_pin(),
                pins.gpio6.into_pull_down_input().into_dyn_pin(),
                pins.gpio7.into_pull_down_input().into_dyn_pin(),
            ];

            let rows: [Row; ROWS] = [
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
            (cols, rows)
        }
        //RIGHT SIDE
        //rows: 22 26 27 20
        //cols: 9 8 7 6 5
        Side::RIGHT => {
            let cols: [Column; COLS] = [
                pins.gpio9.into_pull_down_input().into_dyn_pin(),
                pins.gpio8.into_pull_down_input().into_dyn_pin(),
                pins.gpio7.into_pull_down_input().into_dyn_pin(),
                pins.gpio6.into_pull_down_input().into_dyn_pin(),
                pins.gpio5.into_pull_down_input().into_dyn_pin(),
            ];
            let rows: [Row; ROWS] = [
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
            (cols, rows)
        }
    }
}
//pub type Column<'a> = &'a mut dyn InputPin<Error = Infallible>;

/*
pub fn initialize_split(
    side: Side,
    pins: gpio::bank0::Pins,
) -> ([Column<'static>; COLS], [Row<'static>; ROWS], SideConfig) {
    match side {
        Side::LEFT => {
            let mut col0 = pins.gpio21.into_pull_down_input();
            let mut col1 = pins.gpio4.into_pull_down_input();
            let mut col2 = pins.gpio5.into_pull_down_input();
            let mut col3 = pins.gpio6.into_pull_down_input();
            let mut col4 = pins.gpio7.into_pull_down_input();

            let mut row0 = pins.gpio27.into_push_pull_output();
            let mut row1 = pins.gpio28.into_push_pull_output();
            let mut row2 = pins.gpio26.into_push_pull_output();
            return (
                [&mut col0, &mut col1, &mut col2, &mut col3, &mut col4],
                [&mut row0, &mut row1, &mut row2],
                L_LAYER,
            );
        }
        Side::RIGHT => {
            let mut col0 = pins.gpio21.into_pull_down_input();
            let mut col1 = pins.gpio4.into_pull_down_input();
            let mut col2 = pins.gpio5.into_pull_down_input();
            let mut col3 = pins.gpio6.into_pull_down_input();
            let mut col4 = pins.gpio7.into_pull_down_input();

            let mut row0 = pins.gpio27.into_push_pull_output();
            let mut row1 = pins.gpio28.into_push_pull_output();
            let mut row2 = pins.gpio26.into_push_pull_output();
            return (
                [&mut col0, &mut col1, &mut col2, &mut col3, &mut col4],
                [&mut row0, &mut row1, &mut row2],
                L_LAYER,
            );
        }
    }
}
*/

//pub type Column<'a> = &'a mut dyn InputPin<Error = Infallible>;
pub type Column = Pin<DynPinId, FunctionSioInput, PullDown>;
//pub type Row<'a> = &'a mut dyn OutputPin<Error = Infallible>;
pub type Row = Pin<DynPinId, FunctionSioOutput, PullNone>;
pub type StateMatrix = [[bool; COLS]; ROWS];
pub type SideConfig = [[KeyCode; COLS]; ROWS];

fn scan_keys(rows: &mut [Row; ROWS], cols: &mut [Column; COLS], delay: &mut Delay) -> StateMatrix {
    let mut matrix: StateMatrix = [[false; COLS]; ROWS];
    for (gpio_row, matrix_row) in rows.iter_mut().zip(matrix.iter_mut()) {
        gpio_row.set_high().unwrap();
        delay.delay_us(1);
        for (gpio_col, matrix_col) in cols.iter_mut().zip(matrix_row.iter_mut()) {
            *matrix_col = gpio_col.is_high().unwrap();
        }
        gpio_row.set_low().unwrap();
        delay.delay_us(1);
    }
    matrix
}
pub const L_LAYER: [[KeyCode; COLS]; ROWS] = [[T, R, E, W, Q], [G, F, D, S, A], [B, V, C, X, Z]];
pub const R_LAYER: [[KeyCode; COLS]; ROWS] = [
    [Y, U, I, O, P],
    [H, J, K, L, Semicolon],
    [N, M, Comma, Period, Slash],
];

fn build_report(matrix: &StateMatrix) -> KeyboardReport {
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
                push_key(L_LAYER[row_idx][col_idx] as u8);
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
