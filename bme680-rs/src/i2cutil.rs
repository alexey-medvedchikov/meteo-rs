#![forbid(unsafe_code)]

use embedded_hal::blocking::i2c;

pub enum Error<R, W> {
    Read(R),
    Write(W),
}

pub(crate) fn write<I2C, ADDR, R, W>(
    i2c: &mut I2C,
    address: ADDR,
    bytes: &[u8],
) -> Result<(), Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
    ADDR: Into<i2c::SevenBitAddress>,
{
    i2c.write(address.into(), bytes).map_err(Error::Write)
}

pub(crate) fn read_byte<I2C, ADDR, R, W>(
    i2c: &mut I2C,
    address: ADDR,
    register: u8,
) -> Result<u8, Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
    ADDR: Into<i2c::SevenBitAddress> + Copy,
{
    let mut buf = [0; 1];
    read_bytes(i2c, address, register, &mut buf)?;

    Ok(buf[0])
}

pub(crate) fn read_bytes<I2C, ADDR, R, W>(
    i2c: &mut I2C,
    address: ADDR,
    register: u8,
    buf: &mut [u8],
) -> Result<(), Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
    ADDR: Into<i2c::SevenBitAddress> + Copy,
{
    i2c.write(address.into(), &[register])
        .map_err(Error::Write)?;
    i2c.read(address.into(), buf).map_err(Error::Read)
}
