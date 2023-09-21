#![forbid(unsafe_code)]

pub trait DelayMs {
    fn delay_ms(&mut self, ms: u16);
}

impl<T> DelayMs for &mut T
where
    T: DelayMs + ?Sized,
{
    fn delay_ms(&mut self, ms: u16) {
        T::delay_ms(self, ms);
    }
}
