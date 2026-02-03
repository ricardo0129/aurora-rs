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
        for row in 0..ROWS {
            for col in 0..COLS {
                if self.current_state[row][col] == state[row][col] {
                    continue;
                }
                if state[row][col] {
                    buf.push(KeyEvent::Press {
                        row: (row as u8),
                        col: (col as u8),
                    })
                    .expect("Eventbuffer full, skipping event");
                } else {
                    buf.push(KeyEvent::Release {
                        row: (row as u8),
                        col: (col as u8),
                    })
                    .expect("Eventbuffer full, skipping event");
                }
            }
        }
        buf
    }
}
