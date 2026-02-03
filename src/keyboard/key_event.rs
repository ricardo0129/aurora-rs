pub struct KeyPos {
    row: u8,
    col: u8,
}

#[derive(Copy, Clone)]
pub enum KeyEvent {
    Press { row: u8, col: u8 },
    Release { row: u8, col: u8 },
    None,
}

pub struct EventBuffer<const N: usize> {
    events: [KeyEvent; N],
    len: usize,
}

impl<const N: usize> EventBuffer<N> {
    pub fn new() -> Self {
        Self {
            events: [KeyEvent::None; N],
            len: N,
        }
    }
}
