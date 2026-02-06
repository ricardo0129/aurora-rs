#[derive(Copy, Clone, Debug)]
pub enum KeyEvent {
    Press { row: u8, col: u8 },
    Release { row: u8, col: u8 },
}
