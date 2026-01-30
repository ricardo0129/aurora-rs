use embedded_hal::digital::{InputPin, OutputPin};
use hal::gpio::{DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullNone};
use rp2040_hal as hal;
//use hal::uart::{DataBits, StopBits, UartConfig};
use cortex_m::delay::Delay;
const BAUD: u32 = 9600;
const BIT_US: u32 = 1000000 / BAUD;
pub type TxDataPin = Pin<DynPinId, FunctionSioOutput, PullNone>;
pub type RxDataPin = Pin<DynPinId, FunctionSioInput, PullDown>;

pub fn uart_bitbang_tx(mut byte: u8, serial: &mut TxDataPin, delay: &mut Delay) {
    serial.set_low();
    delay.delay_us(BIT_US);
    for i in 0..8 {
        if byte & 1 > 0 {
            serial.set_high();
        } else {
            serial.set_low();
        }
        delay.delay_us(BIT_US);
        byte >>= 1;
    }
    serial.set_high();
    delay.delay_us(BIT_US);
}

pub fn uart_bitbang_rx(serial: &mut RxDataPin, delay: &mut Delay) -> u8 {
    let mut data: u8 = 0;
    while serial.is_high().expect("couldn't read from pin") {}
    delay.delay_us(BIT_US + BIT_US / 2);
    for i in 0..8 {
        if serial.is_high().expect("couldn't read from pin") {
            data |= 1 << i;
        }
        delay.delay_us(BIT_US);
    }
    delay.delay_us(BIT_US);
    data
}

pub fn hand_shake(serial: &mut TxDataPin, delay: &mut Delay, initiate: bool) -> bool {
    if initiate {
        uart_bitbang_tx(0x55_u8, serial, delay);
        delay.delay_us(100);
    }
    false
}
