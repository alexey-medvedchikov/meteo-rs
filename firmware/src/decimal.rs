use ufmt::{uWrite, Formatter};

pub(crate) struct Decimal<T> {
    n: T,
    d: u8,
}

impl<T> Decimal<T> {
    pub(crate) fn new(n: T, d: u8) -> Self {
        Self { n, d }
    }
}

macro_rules! format_impl {
    ($itype:ty) => {
        impl Decimal<$itype> {
            fn format(&self) -> ([u8; 11], usize) {
                let mut buf = [b'0'; 11];
                let mut pos = 0;
                let mut n = self.n;
                let mut dot = false;

                loop {
                    if pos as u8 == self.d {
                        buf[pos] = b'.';
                        pos += 1;
                        dot = true;
                    }

                    buf[pos] = b'0' + (n % 10) as u8;
                    pos += 1;
                    n /= 10;

                    if dot && n == 0 {
                        break;
                    }
                }

                (buf, pos - 1)
            }
        }
    };
}

macro_rules! udisplay_impl_unsigned {
    ($itype:ty) => {
        impl ufmt::uDisplay for Decimal<$itype> {
            fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
            where
                W: uWrite + ?Sized,
            {
                let (buf, mut pos) = self.format();

                loop {
                    f.write_char(buf[pos] as char)?;
                    if pos == 0 {
                        break;
                    }
                    pos -= 1;
                }

                Ok(())
            }
        }
    };
}

macro_rules! udisplay_impl_signed {
    ($itype:ty, $conv:ty) => {
        impl ufmt::uDisplay for Decimal<$itype> {
            fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
            where
                W: uWrite + ?Sized,
            {
                let (buf, mut pos) = if self.n < 0 {
                    f.write_char('-')?;
                    Decimal::new((-self.n) as $conv, self.d).format()
                } else {
                    self.format()
                };

                loop {
                    f.write_char(buf[pos] as char)?;
                    if pos == 0 {
                        break;
                    }
                    pos -= 1;
                }

                Ok(())
            }
        }
    };
}

macro_rules! udisplay_full_impl_unsigned {
    ($itype:ty) => {
        format_impl!($itype);
        udisplay_impl_unsigned!($itype);
    };
}

macro_rules! udisplay_full_impl_signed {
    ($itype:ty, $conv:ty) => {
        format_impl!($itype);
        udisplay_impl_signed!($itype, $conv);
    };
}

udisplay_full_impl_unsigned!(u8);
udisplay_full_impl_unsigned!(u16);
udisplay_full_impl_unsigned!(u32);
udisplay_full_impl_unsigned!(u64);
udisplay_full_impl_unsigned!(u128);
udisplay_full_impl_unsigned!(usize);
udisplay_full_impl_signed!(i8, u8);
udisplay_full_impl_signed!(i16, u16);
udisplay_full_impl_signed!(i32, u32);
udisplay_full_impl_signed!(i64, u64);
udisplay_full_impl_signed!(i128, u128);
udisplay_full_impl_signed!(isize, usize);

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
            (Decimal::new(1, 1), "0.1"),
            (Decimal::new(1, 0), "1."),
            (Decimal::new(123, 2), "1.23"),
            (Decimal::new(123456, 2), "1234.56"),
            (Decimal::new(-1, 1), "-0.1"),
            (Decimal::new(-1, 0), "-1."),
            (Decimal::new(-123, 2), "-1.23"),
            (Decimal::new(-123456, 2), "-1234.56"),
        ];

        for (decimal, expected) in test_cases {
            let mut buf = StringBuffer::new();
            uwrite!(buf, "{}", decimal).unwrap();
            assert_eq!(buf.as_ref(), expected);
        }
    }
}
