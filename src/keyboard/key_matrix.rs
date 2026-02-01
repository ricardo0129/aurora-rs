use crate::layout;
use cortex_m::delay::Delay;
use rp2040_hal as hal;

use embedded_hal::digital::{InputPin, OutputPin};
use hal::gpio::{DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullNone};

pub type Column = Pin<DynPinId, FunctionSioInput, PullDown>;
pub type Row = Pin<DynPinId, FunctionSioOutput, PullNone>;

pub type StateMatrix = [[bool; layout::COLS]; layout::ROWS];

const PIN_READ_DELAY: u32 = 1;

pub struct KeyMatrix {
    rows: [Row; layout::ROWS],
    cols: [Column; layout::COLS],
}

impl KeyMatrix {}
pub fn scan_keys(
    rows: &mut [Row; layout::ROWS],
    cols: &mut [Column; layout::COLS],
    delay: &mut Delay,
) -> StateMatrix {
    let mut matrix: StateMatrix = [[false; layout::COLS]; layout::ROWS];
    for (gpio_row, matrix_row) in rows.iter_mut().zip(matrix.iter_mut()) {
        gpio_row.set_high().unwrap();
        delay.delay_us(PIN_READ_DELAY);
        for (gpio_col, matrix_col) in cols.iter_mut().zip(matrix_row.iter_mut()) {
            *matrix_col = gpio_col.is_high().unwrap();
        }
        gpio_row.set_low().unwrap();
        delay.delay_us(PIN_READ_DELAY);
    }
    matrix
}
