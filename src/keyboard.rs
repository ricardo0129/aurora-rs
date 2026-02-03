pub mod key_event;
pub mod key_matrix;
pub mod keyboard_state;

use crate::communication;
use crate::keyboard::key_matrix::{Column, Row};
use crate::layout;
use communication::transport::{SerialTransport, Transport};
use cortex_m::delay::Delay;
use key_matrix::KeyMatrix;
use keyboard_state::KeyboardState;
use layout::{Layout, Side};

pub struct Keyboard {
    side: Side,
    pub matrix: KeyMatrix,
    transport: Transport,
    layout: Layout,
    state: KeyboardState,
}

impl Keyboard {
    pub fn new(
        side: Side,
        rows: [Row; layout::ROWS],
        cols: [Column; layout::COLS],
        delay: Delay,
    ) -> Self {
        let mut matrix = KeyMatrix::new(rows, cols, delay);
        Self {
            side,
            matrix,
            layout: Layout::new(&side),
            transport: Transport::Serial(SerialTransport {}),
            state: KeyboardState::new(),
        }
    }

    pub fn tick(&mut self) {
        // 1. Scan local keys
        let local = self.matrix.scan_keys();
        // 2. Exchange state with other half
        // 3. Update merged state
        self.state.update(local);
        // 4. Process layout
        // 5. Emit USB HID / debug / LEDs
    }
}
