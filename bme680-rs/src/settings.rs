#![forbid(unsafe_code)]

use core::convert::TryFrom;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error {
    PowerMode(u8),
    Oversampling(u8),
    IIRFilter(u8),
    HeaterControl(u8),
    NBConversion(u8),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PowerMode {
    Sleep,
    Forced,
}

const POWER_MODE_SLEEP: u8 = 0;
const POWER_MODE_FORCE: u8 = 1;

impl From<PowerMode> for u8 {
    fn from(value: PowerMode) -> Self {
        match value {
            PowerMode::Sleep => POWER_MODE_SLEEP,
            PowerMode::Forced => POWER_MODE_FORCE,
        }
    }
}

impl TryFrom<u8> for PowerMode {
    type Error = Error;

    fn try_from(power_mode: u8) -> Result<Self, Self::Error> {
        match power_mode {
            POWER_MODE_SLEEP => Ok(Self::Sleep),
            POWER_MODE_FORCE => Ok(Self::Forced),
            _ => Err(Error::PowerMode(power_mode)),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Oversampling {
    None = 0,
    X1 = 1,
    X2 = 2,
    X4 = 3,
    X8 = 4,
    X16 = 5,
}

impl TryFrom<u8> for Oversampling {
    type Error = Error;

    fn try_from(os: u8) -> Result<Self, Self::Error> {
        match os {
            0 => Ok(Self::None),
            1 => Ok(Self::X1),
            2 => Ok(Self::X2),
            3 => Ok(Self::X4),
            4 => Ok(Self::X8),
            5 => Ok(Self::X16),
            _ => Err(Error::Oversampling(os)),
        }
    }
}

impl Default for Oversampling {
    fn default() -> Self {
        Self::None
    }
}

/// Infinite Impulse Response filter
/// https://en.wikipedia.org/wiki/Infinite_impulse_response
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IIRFilter {
    S0 = 0,
    S1 = 1,
    S3 = 2,
    S7 = 3,
    S15 = 4,
    S31 = 5,
    S63 = 6,
    S127 = 7,
}

impl TryFrom<u8> for IIRFilter {
    type Error = Error;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::S0),
            1 => Ok(Self::S1),
            2 => Ok(Self::S3),
            3 => Ok(Self::S7),
            4 => Ok(Self::S15),
            5 => Ok(Self::S31),
            6 => Ok(Self::S63),
            7 => Ok(Self::S127),
            _ => Err(Error::IIRFilter(v)),
        }
    }
}

impl Default for IIRFilter {
    fn default() -> Self {
        Self::S0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HeaterControl {
    H0 = 0,
    H1 = 1,
    H2 = 2,
    H3 = 3,
    H4 = 4,
    H5 = 5,
    H6 = 6,
    H7 = 7,
}

impl TryFrom<u8> for HeaterControl {
    type Error = Error;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::H0),
            1 => Ok(Self::H1),
            2 => Ok(Self::H2),
            3 => Ok(Self::H3),
            4 => Ok(Self::H4),
            5 => Ok(Self::H5),
            6 => Ok(Self::H6),
            7 => Ok(Self::H7),
            _ => Err(Error::HeaterControl(v)),
        }
    }
}

impl Default for HeaterControl {
    fn default() -> Self {
        Self::H0
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum NBConversion {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    C6 = 6,
    C7 = 7,
    C8 = 8,
    C9 = 9,
}

impl TryFrom<u8> for NBConversion {
    type Error = Error;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::C0),
            1 => Ok(Self::C1),
            2 => Ok(Self::C2),
            3 => Ok(Self::C3),
            4 => Ok(Self::C4),
            5 => Ok(Self::C5),
            6 => Ok(Self::C6),
            7 => Ok(Self::C7),
            8 => Ok(Self::C8),
            9 => Ok(Self::C9),
            _ => Err(Error::NBConversion(v)),
        }
    }
}

impl Default for NBConversion {
    fn default() -> Self {
        Self::C0
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TemperatureSettings {
    pub humidity_oversampling: Oversampling,
    pub temperature_oversampling: Oversampling,
    pub pressure_oversampling: Oversampling,
    pub iir_filter: IIRFilter,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct GasSettings {
    pub heater_duration_ms: u32,
    pub heater_temperature: u16,
    pub ambient_temperature: i8,
    pub nb_conversion: NBConversion,
    pub heater_control: HeaterControl,
    pub enable_gas_measurement: bool,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Settings {
    pub gas: GasSettings,
    pub temperature: TemperatureSettings,
}
