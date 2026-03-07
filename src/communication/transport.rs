use cortex_m::delay::Delay;
use embedded_hal::digital::{InputPin, OutputPin};
use hal::gpio::{DynPinId, Pin, PullDown};
use rp2040_hal as hal;
use rp2040_hal::gpio::*;
const BAUD: u32 = 9600;
const BIT_US: u32 = 1000000 / BAUD;
const TIMEOUT_US: u32 = 100;
const SERIAL_DELAY: u32 = 32;

pub type TxDataPin = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;

pub enum Transport {
    Serial(SerialTransport),
}

pub struct SerialTransport {
    data_pin: TxDataPin,
}

#[derive(Debug)]
pub enum SerialError {
    Timeout,
    ChecksumMismatch,
}

impl SerialTransport {
    pub fn new(data_pin: TxDataPin) -> Self {
        Self { data_pin }
    }

    pub fn write_byte(&mut self, mut byte: u8, delay: &mut Delay) {
        self.data_pin.set_low();
        delay.delay_us(BIT_US);
        for _ in 0..8 {
            if byte & 1 > 0 {
                self.data_pin.set_high();
            } else {
                self.data_pin.set_low();
            }
            delay.delay_us(BIT_US);
            byte >>= 1;
        }
        self.data_pin.set_high();
        delay.delay_us(BIT_US);
    }

    pub fn read_byte(&mut self, delay: &mut Delay) -> u8 {
        let mut data: u8 = 0;
        let mut rx_pin = self.data_pin.as_input();
        while rx_pin.is_high().expect("couldn't read from pin") {}
        delay.delay_us(BIT_US + BIT_US / 2);
        for i in 0..8 {
            if rx_pin.is_high().expect("couldn't read from pin") {
                data |= 1 << i;
            }
            delay.delay_us(BIT_US);
        }
        delay.delay_us(BIT_US);
        data
    }
}
