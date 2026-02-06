use crate::layout;
use cortex_m::delay::Delay;
use rp2040_hal as hal;

use embedded_hal::digital::{InputPin, OutputPin};
use hal::gpio::{
    DynPinId, FunctionPio0, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullNone,
};

pub type Column = Pin<DynPinId, FunctionSioInput, PullDown>;
pub type Row = Pin<DynPinId, FunctionSioOutput, PullNone>;
pub type OutPin = Pin<DynPinId, FunctionSioOutput, PullDown>;
pub type PioPin = Pin<DynPinId, FunctionPio0, PullDown>;

pub type StateMatrix = [[bool; layout::COLS]; layout::ROWS];

const PIN_READ_DELAY: u32 = 1;

pub struct KeyMatrix {
    rows: [Row; layout::ROWS],
    cols: [Column; layout::COLS],
}

impl KeyMatrix {
    pub fn new(rows: [Row; layout::ROWS], cols: [Column; layout::COLS]) -> Self {
        KeyMatrix { rows, cols }
    }
    pub fn scan_keys(&mut self, delay: &mut Delay) -> StateMatrix {
        let mut matrix: StateMatrix = [[false; layout::COLS]; layout::ROWS];
        for (gpio_row, matrix_row) in self.rows.iter_mut().zip(matrix.iter_mut()) {
            gpio_row.set_high().unwrap();
            delay.delay_us(PIN_READ_DELAY);
            for (gpio_col, matrix_col) in self.cols.iter_mut().zip(matrix_row.iter_mut()) {
                *matrix_col = gpio_col.is_high().unwrap();
            }
            gpio_row.set_low().unwrap();
            delay.delay_us(PIN_READ_DELAY);
        }
        matrix
    }
}
