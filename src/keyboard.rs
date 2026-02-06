pub mod key_event;
pub mod key_matrix;
pub mod keyboard_state;

use crate::communication;
use crate::communication::transport::TxDataPin;
use crate::keyboard::key_matrix::{Column, Row};
use crate::layout::{self, KeyAction};
use communication::transport::{SerialTransport, Transport};
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use heapless::Vec;
use key_matrix::KeyMatrix;
use keyboard_state::KeyboardState;
use layout::{Layout, Side};
use rp2040_hal as hal;

use hal::usb::UsbBus;
use usbd_hid::{descriptor::KeyboardReport, hid_class::HIDClass};

use cortex_m::prelude::_embedded_hal_timer_CountDown;
use rp2040_hal::timer::CountDown;
use usb_device::prelude::UsbDevice;

pub struct Keyboard<'a> {
    side: Side,
    matrix: KeyMatrix,
    transport: Transport,
    layout: Layout,
    state: KeyboardState,
    dev: UsbDevice<'a, rp2040_hal::usb::UsbBus>,
    hid: HIDClass<'a, UsbBus>,
    scan_countdown: CountDown,
    delay: Delay,
}

pub struct KeyboardPins {
    pub rows: [Row; layout::ROWS],
    pub cols: [Column; layout::COLS],
    pub data_pin: TxDataPin,
}

impl<'a> Keyboard<'a> {
    pub fn new(
        side: Side,
        pins: KeyboardPins,
        delay: Delay,
        dev: UsbDevice<'a, rp2040_hal::usb::UsbBus>,
        hid: HIDClass<'a, UsbBus>,
        scan_countdown: CountDown,
    ) -> Self {
        let matrix = KeyMatrix::new(pins.rows, pins.cols);
        Self {
            side,
            matrix,
            layout: Layout::new(&side),
            transport: Transport::Serial(SerialTransport::new(pins.data_pin)),
            state: KeyboardState::new(),
            dev,
            hid,
            scan_countdown,
            delay,
        }
    }

    pub fn tick(&mut self) {
        // 5. Emit USB HID / debug / LEDs

        self.dev.poll(&mut [&mut self.hid]);

        if self.scan_countdown.wait().is_ok() {
            // 1. Scan local keys
            let local = self.matrix.scan_keys(&mut self.delay);
            // 2. Exchange state with other half
            match &mut self.transport {
                Transport::Serial(transport) => match self.side {
                    Side::Left => {
                        transport.write_byte(0x55_u8, &mut self.delay);
                    }
                    Side::Right => {
                        let _byte = transport.read_byte(&mut self.delay);
                    }
                },
            };
            // 3. Update merged state
            let events = self.state.update(local);
            // 4. Process layout
            let key_actions = self.layout.process(events);
            self.process_actions(&key_actions);
            let report = build_report(key_actions);
            self.hid.push_input(&report).ok();
        }
        // drop received data
        self.hid.pull_raw_output(&mut [0; 64]).ok();
    }

    pub fn process_actions(&mut self, key_actions: &Vec<KeyAction, 20>) {
        for action in key_actions {
            match action {
                KeyAction::KeyUp { key: keycode } => {
                    debug!("keycode up: {}", *keycode as u8);
                }
                KeyAction::KeyDown { key: keycode } => {
                    debug!("keycode up: {}", *keycode as u8);
                }
            }
        }
    }
}

fn build_report(key_actions: Vec<KeyAction, 20>) -> KeyboardReport {
    let mut keycodes = [0u8; 6];
    let mut keycode_count = 0;
    let mut push_key = |keycode: u8| {
        keycodes[keycode_count] = keycode;
        keycode_count += 1;
    };
    let modifier = 0;
    for key_action in key_actions {
        if let KeyAction::KeyDown { key: action } = key_action {
            push_key(action as u8);
        }
    }

    KeyboardReport {
        modifier,
        reserved: 0,
        leds: 0,
        keycodes,
    }
}
