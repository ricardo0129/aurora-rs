use crate::keyboard::key_event::KeyEvent;
use crate::keyboard::key_matrix::StateMatrix;
use crate::layout::{COLS, ROWS};
use heapless::Vec;

const MAX_EVENTS: usize = 20;
pub type EventBuffer = Vec<KeyEvent, MAX_EVENTS>;

pub struct KeyboardState {
    current_state: StateMatrix,
}

impl KeyboardState {
    pub fn new() -> Self {
        let current_state = [[false; COLS]; ROWS];
        Self { current_state }
    }

    pub fn update(&mut self, state: StateMatrix) -> EventBuffer {
        let mut buf: Vec<KeyEvent, MAX_EVENTS> = Vec::new();
        for (row_index, row) in state.iter().enumerate() {
            for (col_index, state_value) in row.iter().enumerate() {
                if self.current_state[row_index][col_index] == *state_value {
                    continue;
                }
                if *state_value {
                    buf.push(KeyEvent::Press {
                        row: (row_index as u8),
                        col: (col_index as u8),
                    })
                    .expect("Eventbuffer full, skipping event");
                } else {
                    buf.push(KeyEvent::Release {
                        row: (row_index as u8),
                        col: (col_index as u8),
                    })
                    .expect("Eventbuffer full, skipping event");
                }
            }
        }
        buf
    }
}
