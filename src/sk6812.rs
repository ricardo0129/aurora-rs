use hal::pac::PIO0;
use hal::pio::{Tx, UninitStateMachine};
use rp2040_hal as hal;

pub fn color_as_u32(red: u8, green: u8, blue: u8) -> u32 {
    let color: u32 = ((green as u32) << 16) | ((red as u32) << 8) | (blue as u32);
    color
}

pub type Led = u32;
pub struct LedController<const N: usize> {
    led_state: [Led; N],
    tx: Tx<(PIO0, hal::pio::SM0)>,
}

impl<const N: usize> LedController<N> {
    pub fn new(
        pio: &mut hal::pio::PIO<PIO0>,
        sm0: UninitStateMachine<(PIO0, hal::pio::SM0)>,
        led_pin_id: u8,
        sys_clk: f32,
    ) -> Self {
        let (t1, t2, t3) = (4, 6, 2);
        let bit_freq = 800_000.0 * (t1 + t2 + t3) as f32;
        let div = sys_clk / bit_freq;

        let int = div as u16;
        let frac = (((div - int as f32) * 256.0) + 0.5) as u8;
        let program = pio_proc::pio_asm!(
            ".side_set 1",
            ".define public T1 4",
            ".define public T2 6",
            ".define public T3 2",
            ".wrap_target",
            "bitloop:",
            "   out x, 1        side 0 [T3 - 1]",
            "   jmp !x do_zero  side 1 [T1 - 1]",
            "do_one:",
            "   jmp bitloop     side 1 [T2 - 1]",
            "do_zero:",
            "   nop             side 0 [T2 - 1]",
            ".wrap"
        );
        let installed = pio.install(&program.program).unwrap();
        let (mut sm, _, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .out_shift_direction(rp2040_hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(24)
            .side_set_pin_base(led_pin_id)
            .clock_divisor_fixed_point(int, frac)
            .buffers(hal::pio::Buffers::OnlyTx)
            .build(sm0);
        // The GPIO pin needs to be configured as an output.
        sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
        sm.start();
        Self {
            led_state: [0; N],
            tx,
        }
    }
    pub fn set_pixel(&mut self, idx: usize, color: u32) {
        self.led_state[idx] = color;
    }
    pub fn show(&mut self) {
        for i in 0..N {
            while self.tx.is_full() {}
            self.tx.write(self.led_state[i]);
        }
    }
}
