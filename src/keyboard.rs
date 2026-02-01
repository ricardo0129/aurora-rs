pub mod key_event;
pub mod key_matrix;
pub mod keyboard_state;

use crate::communication;
use crate::layout::Layout;
use communication::transport::Transport;
use key_matrix::KeyMatrix;
use keyboard_state::KeyboardState;

pub enum Side {
    LEFT,
    RIGHT,
}

type Unimplemented = bool;

pub struct Keyboard {
    side: Side,
    matrix: KeyMatrix,
    transport: Transport,
    layout: Layout,
    state: KeyboardState,
}

impl Keyboard {
    fn tick(&mut self) {
        // 1. Scan local keys
        // 2. Exchange state with other half
        // 3. Update merged state
        // 4. Process layout
        // 5. Emit USB HID / debug / LEDs
    }
}
