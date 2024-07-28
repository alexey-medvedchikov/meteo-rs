use core::str;
use ufmt::{uWrite, Formatter};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Decimal<T> {
    sign: bool,
    value: T,
    dot_pos: u8,
}

impl<T> Decimal<T> {
    pub(crate) fn new(value: T, dot_pos: u8) -> Self
    where
        T: PartialOrd + num_traits::Zero,
    {
        Self {
            sign: value.lt(&T::zero()),
            value,
            dot_pos,
        }
    }

    fn new_with_sign(sign: bool, value: T, dot_pos: u8) -> Self {
        Self {
            sign,
            value,
            dot_pos,
        }
    }
}

impl ufmt::uDisplay for Decimal<u32> {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        let mut div = 1;
        for _ in 0..self.dot_pos {
            div *= 10;
        }
        let int = self.value / div;
        let frac = self.value % div;

        let mut buf = [b'.'; 16];
        let mut i = buf.len() - 1;
        let (mut n, mut last) = if self.dot_pos == 0 {
            (int, true)
        } else {
            (frac, false)
        };

        loop {
            buf[i] = (n % 10) as u8 + b'0';
            n /= 10;
            i -= 1;

            if n == 0 {
                if last {
                    break;
                }
                i -= 1;
                n = int;
                last = true;
            }
        }

        if self.sign {
            buf[i] = b'-';
        } else {
            i += 1;
        }

        f.write_str(unsafe { str::from_utf8_unchecked(&buf[i..]) })?;
        Ok(())
    }
}

impl ufmt::uDisplay for Decimal<i16> {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        Decimal::new_with_sign(self.sign, self.value.unsigned_abs() as u32, self.dot_pos).fmt(f)
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
            (Decimal::new(1u32, 0), "1"),
            (Decimal::new(1, 1), "0.1"),
            (Decimal::new(123, 2), "1.23"),
            (Decimal::new(12345, 2), "123.45"),
        ];

        for (decimal, expected) in test_cases {
            let mut buf = StringBuffer::new();
            uwrite!(buf, "{}", decimal).unwrap();
            assert_eq!(buf.as_ref(), expected);
        }

        let test_cases = vec![
            (Decimal::new(1i16, 0), "1"),
            (Decimal::new(1, 1), "0.1"),
            (Decimal::new(123, 2), "1.23"),
            (Decimal::new(12345, 2), "123.45"),
            (Decimal::new(-1, 0), "-1"),
            (Decimal::new(-1, 1), "-0.1"),
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
