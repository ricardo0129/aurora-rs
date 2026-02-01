const MATRIX_ROWS: usize = 4;
const MATRIX_COLS: usize = 5;

pub type MatrixRow = [u8; MATRIX_COLS];
pub struct KeyMatrix {
    pub rows: [MatrixRow; MATRIX_ROWS],
}
