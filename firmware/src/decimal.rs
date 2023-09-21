use core::{mem::MaybeUninit, slice, str};
use ufmt::{uWrite, Formatter};

pub(crate) struct Decimal<T> {
    value: T,
    dot_pos: u8,
}

impl<T> Decimal<T> {
    pub(crate) fn new(value: T, dot_pos: u8) -> Self {
        Self { value, dot_pos }
    }
}

impl ufmt::uDisplay for Decimal<u32> {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        if self.dot_pos == 0 {
            return ufmt::uDisplay::fmt(&self.value, f);
        }

        let div = u32::pow(10, self.dot_pos as u32);
        let int = self.value / div;
        let frac = self.value % div;

        let mut buf = [MaybeUninit::<u8>::uninit(); 16];
        let ptr = buf.as_mut_ptr().cast::<u8>();
        let len = buf.len();

        let mut n = frac;
        let mut i = len - 1;

        loop {
            unsafe { ptr.add(i).write((n % 10) as u8 + b'0') }
            n /= 10;
            i -= 1;

            if n == 0 {
                break;
            }
        }

        unsafe { ptr.add(i).write(b'.') }
        i -= 1;

        n = int;

        loop {
            unsafe { ptr.add(i).write((n % 10) as u8 + b'0') }
            n /= 10;
            i -= 1;

            if n == 0 {
                break;
            }
        }

        let s =
            unsafe { str::from_utf8_unchecked(slice::from_raw_parts(ptr.add(i + 1), len - i - 1)) };

        f.write_str(s)?;
        Ok(())
    }
}

impl ufmt::uDisplay for Decimal<i16> {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        if self.dot_pos == 0 {
            return ufmt::uDisplay::fmt(&self.value, f);
        }

        let mut value = self.value;
        let mut sign = false;
        if value < 0 {
            sign = true;
            value = -value;
        }

        let div = i16::pow(10, self.dot_pos as u32);
        let int = value / div;
        let frac = value % div;

        let mut buf = [MaybeUninit::<u8>::uninit(); 16];
        let ptr = buf.as_mut_ptr().cast::<u8>();
        let len = buf.len();

        let mut n = frac;
        let mut i = len - 1;

        loop {
            unsafe { ptr.add(i).write((n % 10) as u8 + b'0') }
            n /= 10;
            i -= 1;

            if n == 0 {
                break;
            }
        }

        unsafe { ptr.add(i).write(b'.') }
        i -= 1;

        n = int;

        loop {
            unsafe { ptr.add(i).write((n % 10) as u8 + b'0') }
            n /= 10;
            i -= 1;

            if n == 0 {
                break;
            }
        }

        if sign {
            unsafe { ptr.add(i).write(b'-') }
            i -= 1;
        }

        let s =
            unsafe { str::from_utf8_unchecked(slice::from_raw_parts(ptr.add(i + 1), len - i - 1)) };

        f.write_str(s)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use crate::decimal::Decimal;
    use alloc::{string::String, vec};
    use core::fmt::Write;
    use ufmt::{uWrite, uwrite};

    #[derive(Debug)]
    struct StringBuffer(String);

    impl StringBuffer {
        fn new() -> Self {
            Self(String::from(""))
        }

        fn as_ref(&self) -> &str {
            self.0.as_ref()
        }
    }

    impl uWrite for StringBuffer {
        type Error = core::fmt::Error;

        fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
            self.0.write_str(s)
        }
    }

    #[test]
    fn uwrite_impl() {
        let test_cases = vec![
            (Decimal::new(1u32, 1), "0.1"),
            (Decimal::new(1, 0), "1"),
            (Decimal::new(123, 2), "1.23"),
            (Decimal::new(12345, 2), "123.45"),
        ];

        for (decimal, expected) in test_cases {
            let mut buf = StringBuffer::new();
            uwrite!(buf, "{}", decimal).unwrap();
            assert_eq!(buf.as_ref(), expected);
        }

        let test_cases = vec![
            (Decimal::new(1i16, 1), "0.1"),
            (Decimal::new(1, 0), "1"),
            (Decimal::new(123, 2), "1.23"),
            (Decimal::new(12345, 2), "123.45"),
            (Decimal::new(-1, 1), "-0.1"),
            (Decimal::new(-1, 0), "-1"),
            (Decimal::new(-123, 2), "-1.23"),
            (Decimal::new(-12345, 2), "-123.45"),
        ];

        for (decimal, expected) in test_cases {
            let mut buf = StringBuffer::new();
            uwrite!(buf, "{}", decimal).unwrap();
            assert_eq!(buf.as_ref(), expected);
        }
    }
}
