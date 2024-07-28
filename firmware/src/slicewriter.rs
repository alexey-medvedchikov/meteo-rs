use core::str;
use ufmt::uWrite;

pub enum WriteError {
    OutOfBounds,
}

pub(crate) struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub(crate) fn position(&self) -> usize {
        self.pos
    }
}

impl<'a> uWrite for SliceWriter<'a> {
    type Error = WriteError;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        let src = s.as_bytes();
        if self.buf.len() - self.pos < src.len() {
            self.buf[self.pos..self.pos + src.len()].copy_from_slice(src);
            self.pos += src.len();
            Err(WriteError::OutOfBounds)
        } else {
            Ok(())
        }
    }

    fn write_char(&mut self, c: char) -> Result<(), Self::Error> {
        if self.buf.len() > self.pos {
            let mut dst = [0u8; 4];
            self.write_str(c.encode_utf8(&mut dst))
        } else {
            Err(WriteError::OutOfBounds)
        }
    }
}
