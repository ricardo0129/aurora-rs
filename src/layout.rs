use crate::key_codes::KeyCode::{self, *};

#[derive(Copy, Clone)]
pub enum Side {
    LEFT,
    RIGHT,
}
pub const ROWS: usize = 3;
pub const COLS: usize = 5;

pub type Layer = [[KeyCode; COLS]; ROWS];

pub const L_LAYER: Layer = [[T, R, E, W, Q], [G, F, D, S, A], [B, V, C, X, Z]];
pub const R_LAYER: Layer = [
    [Y, U, I, O, P],
    [H, J, K, L, Semicolon],
    [N, M, Comma, Period, Slash],
];

pub struct Layout {
    layers: [Layer; 1],
}

impl Layout {
    pub fn new(side: &Side) -> Self {
        match side {
            Side::LEFT => Layout {
                layers: [L_LAYER; 1],
            },
            Side::RIGHT => Layout {
                layers: [R_LAYER; 1],
            },
        }
    }
}
