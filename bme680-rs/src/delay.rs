#![forbid(unsafe_code)]

pub trait DelayMs {
    fn delay_ms(&mut self, ms: u16);
}
