use ufmt::uWrite;

pub enum WriteError {
    OutOfBounds,
}

pub(crate) struct StringWriter<'a, const N: usize> {
    buf: &'a mut [u8; N],
    pos: usize,
}

impl<'a, const N: usize> StringWriter<'a, N> {
    pub(crate) fn new(buf: &'a mut [u8; N]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl<'a, const N: usize> uWrite for StringWriter<'a, N> {
    type Error = WriteError;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        if self.pos + s.len() < self.buf.len() {
            for c in s.chars() {
                self.write_char(c)?;
            }
            Ok(())
        } else {
            Err(WriteError::OutOfBounds)
        }
    }

    fn write_char(&mut self, c: char) -> Result<(), Self::Error> {
        if self.pos < self.buf.len() {
            self.buf[self.pos] = c as u8;
            self.pos += 1;
            Ok(())
        } else {
            Err(WriteError::OutOfBounds)
        }
    }
}
