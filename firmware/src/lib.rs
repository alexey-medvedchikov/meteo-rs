#![cfg_attr(not(test), no_std)]

mod decimal;
mod stringwriter;

use crate::decimal::Decimal;
use crate::stringwriter::{StringWriter, WriteError};

use ascii::{AsAsciiStrError, AsciiStr};
use bme680_rs::{
    Bme680, GasSettings, I2CAddress, IIRFilter, Oversampling, PowerMode, Readings, Settings,
    TemperatureSettings,
};
use display_interface::DisplayError;
use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::blocking::{delay::DelayMs, i2c};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
use ufmt::uwriteln;

type Display<I2C, SIZE> = Ssd1306<I2CInterface<I2C>, SIZE, BufferedGraphicsMode<SIZE>>;
type Sensor<'a, I2C> = Bme680<'a, I2C>;
type BufferedDisplaySize = DisplaySize128x64;

const STEPS_SMOOTH: usize = 10;

pub fn run<I2C, D, R, W, F>(
    display: &mut Display<I2C, BufferedDisplaySize>,
    sensor: &mut Sensor<I2C>,
    delay: &mut D,
    mut callback: F,
) -> Result<(), Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
    D: DelayMs<u8>,
    F: FnMut(&mut D),
{
    let text_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

    loop {
        let mut gas_resist_vec = [0; STEPS_SMOOTH];
        let mut last_data = Readings::default();

        let mut step = 0;
        while step < STEPS_SMOOTH {
            sensor.set_power_mode(delay, PowerMode::Forced)?;
            let (data, state) = sensor.get_readings()?;

            if state == bme680_rs::ReadingsCondition::Changed {
                gas_resist_vec[step] = data.gas_resistance();
                last_data = data;
                step += 1;
            }

            callback(delay);
        }

        let pressure = last_data.pressure();
        let temp = last_data.temperature();
        let humidity = last_data.humidity();
        let gas_resist = gas_resist_vec.iter().sum::<u32>() / gas_resist_vec.len() as u32;

        let aqi = air_quality_index(humidity, gas_resist);
        let aqi = Decimal::new(aqi, 3);
        let temp = Decimal::new(temp, 2);
        let humidity = Decimal::new(humidity, 3);

        let mut buf = [b' '; 128];
        let msg = {
            let mut writer = StringWriter::new(&mut buf);
            uwriteln!(writer, "Temp: {} C", temp)?;
            uwriteln!(writer, "Pres: {} Pa", pressure)?;
            uwriteln!(writer, "Humd: {} %", humidity)?;
            uwriteln!(writer, "AQI:  {}", aqi)?;
            uwriteln!(writer, "Gas:  {} Ohm", gas_resist)?;
            AsciiStr::from_ascii(&buf)?.as_str()
        };

        display.clear_buffer();
        Text::with_baseline(msg, Point::zero(), text_style, Baseline::Top).draw(display)?;
        display.flush()?;
    }
}

pub fn display_acquire<I2C, R, W>(
    i2c: I2C,
) -> Result<Display<I2C, BufferedDisplaySize>, Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
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

pub fn sensor_acquire<'a, I2C, D, R, W>(
    i2c: &'a mut I2C,
    delay: &mut D,
) -> Result<Sensor<'a, I2C>, Error<R, W>>
where
    I2C: i2c::Read<Error = R> + i2c::Write<Error = W>,
    D: DelayMs<u8>,
{
    let mut sensor = Bme680::new(i2c, I2CAddress::SECONDARY)?;
    sensor.set_settings(
        delay,
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

pub enum Error<R, W> {
    Sensor(bme680_rs::Error<R, W>),
    Display(DisplayError),
    AsAsciiStr(AsAsciiStrError),
    OutOfBounds(WriteError),
    Peripherals,
}

impl<R, W> From<bme680_rs::Error<R, W>> for Error<R, W> {
    fn from(value: bme680_rs::Error<R, W>) -> Self {
        Error::Sensor(value)
    }
}

impl<R, W> From<DisplayError> for Error<R, W> {
    fn from(value: DisplayError) -> Self {
        Error::Display(value)
    }
}

impl<R, W> From<AsAsciiStrError> for Error<R, W> {
    fn from(value: AsAsciiStrError) -> Self {
        Error::AsAsciiStr(value)
    }
}

impl<R, W> From<WriteError> for Error<R, W> {
    fn from(value: WriteError) -> Self {
        Error::OutOfBounds(value)
    }
}

fn air_quality_index(humidity: u32, gas_resist: u32) -> u32 {
    let humidity_ref = 40;
    let gas_lower_limit = 5000;
    let gas_upper_limit = 50000;

    let humidity_score = if (38000..=42000).contains(&humidity) {
        25000
    } else if humidity < 38000 {
        humidity * 25 / humidity_ref
    } else {
        (4166660 - 41666 * humidity_ref - 25 * humidity) / (100 - humidity_ref)
    };

    let gas_resist = gas_resist.clamp(gas_lower_limit, gas_upper_limit);
    let gas_score = (gas_resist - gas_lower_limit) * 75000 / (gas_upper_limit - gas_lower_limit);

    humidity_score + gas_score
}

#[cfg(test)]
mod test {
    use crate::air_quality_index;

    #[test]
    fn air_quality_index_test() {
        let tests = [
            (air_quality_index(30000, 1000), 18750),
            (air_quality_index(40000, 1000), 25000),
            (air_quality_index(50000, 1000), 20833),
            (air_quality_index(30000, 10000), 27083),
            (air_quality_index(40000, 10000), 33333),
            (air_quality_index(50000, 10000), 29166),
            (air_quality_index(30000, 100000), 93750),
            (air_quality_index(40000, 100000), 100000),
            (air_quality_index(50000, 100000), 95833),
        ];

        for (value, expected) in tests {
            assert_eq!(value.abs_diff(expected), 0);
        }
    }
}
