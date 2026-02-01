pub mod key_event;
pub mod key_matrix;

pub enum Side {
    LEFT,
    RIGHT,
}

type Unimplemented = bool;

pub struct Keyboard {
    side: Side,
    matrix: Unimplemented,
    transport: Unimplemented,
    layout: Unimplemented,
    state: Unimplemented,
}
