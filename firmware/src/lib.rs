#![cfg_attr(not(test), no_std)]

mod byteswriter;
mod decimal;

use crate::decimal::Decimal;

use bme680_rs::{
    air_quality_index, Bme680, GasSettings, IIRFilter, Oversampling, PowerMode, Readings, Settings,
    TemperatureSettings, I2C_SECONDARY_ADDR,
};
use display_interface::DisplayError;
use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::i2c;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
use ufmt::uwrite;

type Display<I2C, SIZE> = Ssd1306<I2CInterface<I2C>, SIZE, BufferedGraphicsMode<SIZE>>;
type Sensor<'a, I2C> = Bme680<'a, I2C>;
type BufferedDisplaySize = DisplaySize128x64;

const STEPS_SMOOTH: usize = 10;

pub fn run<I2C, D, F>(
    display: &mut Display<I2C, BufferedDisplaySize>,
    sensor: &mut Sensor<I2C>,
    delay: &mut D,
    mut callback: F,
) -> Result<(), Error<I2C::Error>>
where
    I2C: i2c::I2c,
    D: DelayMs,
    F: FnMut(&mut D),
{
    let text_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

    loop {
        let mut samples = [Readings::default(); STEPS_SMOOTH];
        let mut samples_num = 0;

        for _ in 0..STEPS_SMOOTH {
            sensor.set_power_mode(&mut DelayMsWrapper(delay), PowerMode::Forced)?;
            let (data, state) = sensor.get_readings()?;

            if state == bme680_rs::ReadingsCondition::Changed {
                samples[samples_num] = data;
                samples_num += 1;
            }

            callback(delay);
        }

        let mut pressure = 0;
        let mut temperature = 0;
        let mut humidity = 0;
        let mut gas_resistance = 0;

        for data in &samples[..samples_num] {
            pressure += data.pressure();
            temperature += data.temperature() as i32;
            humidity += data.humidity();
            gas_resistance += data.gas_resistance();
        }

        let humidity = humidity / samples_num as u32;
        let gas_resistance = gas_resistance / samples_num as u32;

        let mut buf = [b' '; 128];
        let mut w = byteswriter::BytesWriter::new(&mut buf);
        uwrite!(
            w,
            "Temp: {} C\nPres: {} Pa\nHumd: {} %\nAQI:  {}\nGas:  {} Ohm",
            Decimal::new((temperature / samples_num as i32) as i16, 2),
            pressure / samples_num as u32,
            Decimal::new(humidity, 3),
            Decimal::new(air_quality_index(humidity, gas_resistance), 3),
            gas_resistance,
        )?;

        display.clear_buffer();
        Text::with_baseline(w.as_str(), Point::zero(), text_style, Baseline::Top).draw(display)?;
        display.flush()?;
    }
}

pub fn display_acquire<I2C>(
    i2c: I2C,
) -> Result<Display<I2C, BufferedDisplaySize>, Error<I2C::Error>>
where
    I2C: i2c::I2c,
{
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        BufferedDisplaySize {},
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init()?;

    Ok(display)
}

pub fn sensor_acquire<'a, I2C, D>(
    i2c: &'a mut I2C,
    delay: &mut D,
) -> Result<Sensor<'a, I2C>, Error<I2C::Error>>
where
    I2C: i2c::I2c,
    D: DelayMs,
{
    let mut sensor = Bme680::new(i2c, I2C_SECONDARY_ADDR)?;
    sensor.set_settings(
        &mut DelayMsWrapper(delay),
        Settings {
            gas: GasSettings {
                enable_gas_measurement: true,
                heater_temperature: 300,
                heater_duration_ms: 100,
                ambient_temperature: 24,
                ..Default::default()
            },
            temperature: TemperatureSettings {
                humidity_oversampling: Oversampling::X16,
                temperature_oversampling: Oversampling::X16,
                pressure_oversampling: Oversampling::X16,
                iir_filter: IIRFilter::S3,
            },
        },
    )?;

    Ok(sensor)
}

#[derive(Clone, Debug)]
pub enum Error<BusError> {
    Sensor(bme680_rs::Error<BusError>),
    Display(DisplayError),
    OutOfBounds(byteswriter::OutOfBoundsError),
}

impl<BusError> From<bme680_rs::Error<BusError>> for Error<BusError> {
    fn from(value: bme680_rs::Error<BusError>) -> Self {
        Error::Sensor(value)
    }
}

impl<BusError> From<DisplayError> for Error<BusError> {
    fn from(value: DisplayError) -> Self {
        Error::Display(value)
    }
}

impl<BusError> From<byteswriter::OutOfBoundsError> for Error<BusError> {
    fn from(value: byteswriter::OutOfBoundsError) -> Self {
        Error::OutOfBounds(value)
    }
}

pub trait DelayMs {
    fn delay_ms(&mut self, ms: u16);
}

impl<T> DelayMs for &mut T
where
    T: bme680_rs::DelayMs + ?Sized,
{
    fn delay_ms(&mut self, ms: u16) {
        T::delay_ms(self, ms);
    }
}

struct DelayMsWrapper<'a, T>(&'a mut T);

impl<'a, T: DelayMs> bme680_rs::DelayMs for DelayMsWrapper<'a, T> {
    fn delay_ms(&mut self, ms: u16) {
        self.0.delay_ms(ms)
    }
}
