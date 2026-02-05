use heapless::Vec;

use crate::{
    key_codes::KeyCode::{self, *},
    keyboard::{key_event::KeyEvent, keyboard_state::EventBuffer},
};

#[derive(Copy, Clone)]
pub enum Side {
    LEFT,
    RIGHT,
}

#[derive(Debug)]
pub enum KeyAction {
    KeyDown { key: KeyCode },
    KeyUp { key: KeyCode },
    LayerPush { layer: u8 },
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
    current_layer: usize,
}

impl Layout {
    pub fn new(side: &Side) -> Self {
        match side {
            Side::LEFT => Layout {
                layers: [L_LAYER; 1],
                current_layer: 0,
            },
            Side::RIGHT => Layout {
                layers: [R_LAYER; 1],
                current_layer: 0,
            },
        }
    }
    pub fn process(&mut self, key_events: EventBuffer) -> Vec<KeyAction, 20> {
        let mut events: Vec<KeyAction, 20> = Vec::new();
        for event in key_events {
            match event {
                KeyEvent::Press { row, col } => {
                    events
                        .push(KeyAction::KeyDown {
                            key: self.layers[0][row as usize][col as usize],
                        })
                        .unwrap();
                }
                KeyEvent::Release { row, col } => {}
                KeyEvent::None => {}
            }
        }
        events
    }
}
