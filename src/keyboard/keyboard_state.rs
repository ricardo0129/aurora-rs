use crate::keyboard::key_event::{EventBuffer, KeyEvent};
use crate::keyboard::key_matrix::StateMatrix;
use crate::layout::{COLS, ROWS};

const MAX_EVENTS: usize = 20;

pub struct KeyboardState {
    current_state: StateMatrix,
}

impl KeyboardState {
    pub fn new() -> Self {
        let current_state = [[false; COLS]; ROWS];
        Self { current_state }
    }

    pub fn update(&mut self, state: StateMatrix) -> EventBuffer<MAX_EVENTS> {
        let mut buf: EventBuffer<MAX_EVENTS> = EventBuffer::new();
        buf
    }
}
