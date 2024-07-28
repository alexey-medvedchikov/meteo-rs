use core::str;
use ufmt::uWrite;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutOfBoundsError;

#[derive(Debug)]
pub(crate) struct BytesWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BytesWriter<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub(crate) fn as_str(&self) -> &str {
        // SAFETY: all BytesWriter::write_* methods accept valid UTF-8 input only, so buf can't
        // contain any invalid sequences. It is safe to return string unchecked.
        unsafe { str::from_utf8_unchecked(&self.buf[..self.pos]) }
    }
}

impl<'a> uWrite for BytesWriter<'a> {
    type Error = OutOfBoundsError;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        let src = s.as_bytes();
        let remaining_capacity = self.buf.len() - self.pos;
        if remaining_capacity >= src.len() {
            self.buf[self.pos..self.pos + src.len()].copy_from_slice(src);
            self.pos += src.len();
            Ok(())
        } else {
            Err(OutOfBoundsError)
        }
    }

    fn write_char(&mut self, c: char) -> Result<(), Self::Error> {
        if self.buf.len() > self.pos {
            let mut dst = [0u8; 4];
            self.write_str(c.encode_utf8(&mut dst))
        } else {
            Err(OutOfBoundsError)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn write_str() {
        let mut buf = [0; 16];
        let mut w = BytesWriter::new(&mut buf);
        assert_eq!(Ok(()), w.write_str("Hello, world!"));
        assert_eq!("Hello, world!", w.as_str());
        assert_eq!(Ok(()), w.write_str("123"));
        assert_eq!("Hello, world!123", w.as_str());
        assert_eq!(Err(OutOfBoundsError), w.write_str("Hello"));
    }

    #[test]
    fn write_char() {
        let mut buf = [0; 2];
        let mut w = BytesWriter::new(&mut buf);
        assert_eq!(Ok(()), w.write_char('A'));
        assert_eq!("A", w.as_str());
        assert_eq!(Ok(()), w.write_char('B'));
        assert_eq!("AB", w.as_str());
        assert_eq!(Err(OutOfBoundsError), w.write_char('C'));
    }
}
