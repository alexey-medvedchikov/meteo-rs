#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod aqi;
mod calc;
mod delay;
mod settings;

pub use self::aqi::air_quality_index;
pub use self::delay::DelayMs;
pub use self::settings::{
    GasSettings, HeaterControl, IIRFilter, NBConversion, Oversampling, PowerMode, Settings,
    TemperatureSettings,
};
use embedded_hal::i2c;

const ADDR_RES_HEAT_VAL_ADDR: u8 = 0x00;
const ADDR_RES_HEAT_RANGE_ADDR: u8 = 0x02;
const ADDR_RANGE_SW_ERR_ADDR: u8 = 0x04;
const FIELD0_ADDR: u8 = 0x1d;
const ADDR_SENS_CONF_START: u8 = 0x5A;
const RES_HEAT0_ADDR: u8 = 0x5A;
const ADDR_GAS_CONF_START: u8 = 0x64;
const GAS_WAIT0_ADDR: u8 = 0x64;
const CONF_ADDR_START: u8 = 0x70;
const CONF_HEAT_CTRL_ADDR: u8 = 0x70;
const CONF_ODR_RUN_GAS_NBC_ADDR: u8 = 0x71;
const CONF_OS_H_ADDR: u8 = 0x72;
const CONF_POWER_MODE_ADDR: u8 = 0x74;
const CONF_ODR_FILTER_ADDR: u8 = 0x75;
const COEFF_ADDR1: u8 = 0x89;
const COEFF_ADDR1_LEN: usize = 25;
const CHIP_ID_ADDR: u8 = 0xD0;
const SOFT_RESET_ADDR: u8 = 0xE0;
const COEFF_ADDR2: u8 = 0xE1;
const COEFF_ADDR2_LEN: usize = 16;

const MODE_MASK: u8 = 0x03;
const RSERROR_MASK: u8 = 0xf0;
const NEW_DATA_MASK: u8 = 0x80;
const GAS_RANGE_MASK: u8 = 0x0f;
const GASM_VALID_MASK: u8 = 0x20;
const HEAT_STAB_MASK: u8 = 0x10;

const POLL_PERIOD_MS: u16 = 10;
const CHIP_ID: u8 = 0x61;
const SOFT_RESET_CMD: u8 = 0xb6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error<BusError> {
    BusError(BusError),
    BoundaryCheckFailure(&'static str),
    Settings(settings::Error),
    DeviceNotFound,
}

impl<BusError> From<settings::Error> for Error<BusError> {
    fn from(value: settings::Error) -> Self {
        Self::Settings(value)
    }
}

/// I2C Slave Address
/// To determine the slave address of your device you can use `i2cdetect -y 1` on linux.
/// The 7-bit device address is 111011x. The 6 MSB bits are fixed.
/// The last bit is changeable by SDO value and can be changed during operation.
/// Connecting SDO to GND results in slave address 1110110 (0x76); connecting it to V DDIO results in slave
/// address 1110111 (0x77), which is the same as BMP280’s I2C address.
pub const I2C_PRIMARY_ADDR: i2c::SevenBitAddress = 0x76;
pub const I2C_SECONDARY_ADDR: i2c::SevenBitAddress = 0x77;

/// Calibration data used during initialization
#[derive(Clone, Copy, Debug, Default)]
pub struct CalibrationData {
    par_h1: u16,
    par_h2: u16,
    par_h3: i8,
    par_h4: i8,
    par_h5: i8,
    par_h6: u8,
    par_h7: i8,
    par_gh1: i8,
    par_gh2: i16,
    par_gh3: i8,
    par_t1: u16,
    par_t2: i16,
    par_t3: i8,
    par_p1: u16,
    par_p2: i16,
    par_p3: i8,
    par_p4: i16,
    par_p5: i16,
    par_p6: i8,
    par_p7: i8,
    par_p8: i16,
    par_p9: i16,
    par_p10: u8,
    res_heat_range: u8,
    res_heat_val: i8,
    range_sw_err: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReadingsCondition {
    Changed,
    Unchanged,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Readings {
    status: u8,
    temperature: i16,
    pressure: u32,
    humidity: u32,
    gas_resistance: u32,
}

impl Readings {
    /// Temperature in degree Celsius (°C) as returned by the sensor
    pub fn temperature(&self) -> i16 {
        self.temperature
    }

    /// Pressure in hectopascal (hPA) as returned by the sensor
    pub fn pressure(&self) -> u32 {
        self.pressure
    }

    /// Humidity in % relative humidity as returned by the sensor
    pub fn humidity(&self) -> u32 {
        self.humidity
    }

    /// Gas resistance in ohms
    pub fn gas_resistance(&self) -> u32 {
        self.gas_resistance
    }

    /// Whether a real (and not a dummy) gas reading was performed.
    pub fn is_valid_gas_reading(&self) -> bool {
        self.status & GASM_VALID_MASK != 0
    }

    /// Whether the heater target temperature for the gas reading was reached.
    ///
    /// If this values is `false`, the heating duration was likely too short or
    /// the target temperature too high.
    pub fn is_heater_temperature_reached(&self) -> bool {
        self.status & HEAT_STAB_MASK != 0
    }
}

#[derive(Debug)]
pub struct Bme680<'a, I2C> {
    i2c: &'a mut I2C,
    address: i2c::SevenBitAddress,
    calibration_data: CalibrationData,
}

impl<'a, I2C> Bme680<'a, I2C>
where
    I2C: i2c::I2c,
{
    pub fn new(i2c: &'a mut I2C, address: i2c::SevenBitAddress) -> Result<Self, Error<I2C::Error>> {
        let chip_id = read_byte(i2c, address, CHIP_ID_ADDR).map_err(Error::BusError)?;

        if chip_id == CHIP_ID {
            let calibration_data = Self::get_calibration_data(i2c, address)?;
            Ok(Self {
                i2c,
                address,
                calibration_data,
            })
        } else {
            Err(Error::DeviceNotFound)
        }
    }

    pub fn soft_reset<D>(&mut self, delay: &mut D) -> Result<(), I2C::Error>
    where
        D: DelayMs,
    {
        self.i2c
            .write(self.address, &[SOFT_RESET_ADDR, SOFT_RESET_CMD])?;
        delay.delay_ms(POLL_PERIOD_MS);
        Ok(())
    }

    fn set_iir_filter(&mut self, settings: &TemperatureSettings) -> Result<(), I2C::Error> {
        let data = read_byte(self.i2c, self.address, CONF_ODR_FILTER_ADDR)?;
        let data = (data & !0x1c) | (((settings.iir_filter as u8) << 2) & 0x1c);
        self.i2c.write(self.address, &[CONF_ODR_FILTER_ADDR, data])
    }

    fn set_heater_control(&mut self, settings: &GasSettings) -> Result<(), I2C::Error> {
        let data = read_byte(self.i2c, self.address, CONF_HEAT_CTRL_ADDR)?;
        let data = (data & !0x8) | (settings.heater_control as u8 & 0x8);
        self.i2c.write(self.address, &[CONF_HEAT_CTRL_ADDR, data])
    }

    fn set_temperature_and_humidity_oversampling(
        &mut self,
        settings: &TemperatureSettings,
    ) -> Result<(), I2C::Error> {
        let data = read_byte(self.i2c, self.address, CONF_POWER_MODE_ADDR)?;
        let data = (data & !0xe0) | (((settings.temperature_oversampling as u8) << 5) & 0xe0);
        let data = (data & !0x1c) | (((settings.pressure_oversampling as u8) << 2) & 0x1c);
        self.i2c.write(self.address, &[CONF_POWER_MODE_ADDR, data])
    }

    fn set_humidity_oversampling(
        &mut self,
        settings: &TemperatureSettings,
    ) -> Result<(), I2C::Error> {
        let data = read_byte(self.i2c, self.address, CONF_OS_H_ADDR)?;
        let data = (data & !0x7) | (settings.humidity_oversampling as u8 & 0x7);
        self.i2c.write(self.address, &[CONF_OS_H_ADDR, data])
    }

    fn set_run_gas_and_nb_conversion(&mut self, settings: &GasSettings) -> Result<(), I2C::Error> {
        let data = read_byte(self.i2c, self.address, CONF_ODR_RUN_GAS_NBC_ADDR)?;
        let data = (data & !0x10) | (((settings.enable_gas_measurement as u8) << 4) & 0x10);
        let data = (data & !0xf) | (settings.nb_conversion as u8 & 0xf);
        self.i2c
            .write(self.address, &[CONF_ODR_RUN_GAS_NBC_ADDR, data])
    }

    fn set_gas(&mut self, settings: GasSettings) -> Result<(), I2C::Error> {
        let heater_resistance = calc::heater_temperature(
            &self.calibration_data,
            settings.ambient_temperature,
            settings.heater_temperature,
        );
        self.i2c
            .write(self.address, &[RES_HEAT0_ADDR, heater_resistance])?;
        let heater_duration = calc::heater_duration(settings.heater_duration_ms);
        self.i2c
            .write(self.address, &[GAS_WAIT0_ADDR, heater_duration])
    }

    pub fn set_settings<D>(
        &mut self,
        delay: &mut D,
        settings: Settings,
    ) -> Result<(), Error<I2C::Error>>
    where
        D: DelayMs,
    {
        self.set_power_mode(delay, PowerMode::Forced)?;

        self.set_gas(settings.gas).map_err(Error::BusError)?;
        self.set_iir_filter(&settings.temperature)
            .map_err(Error::BusError)?;
        self.set_heater_control(&settings.gas)
            .map_err(Error::BusError)?;
        self.set_temperature_and_humidity_oversampling(&settings.temperature)
            .map_err(Error::BusError)?;
        self.set_humidity_oversampling(&settings.temperature)
            .map_err(Error::BusError)?;
        self.set_run_gas_and_nb_conversion(&settings.gas)
            .map_err(Error::BusError)?;

        self.set_power_mode(delay, PowerMode::Sleep)?;

        Ok(())
    }

    pub fn get_settings(&mut self) -> Result<Settings, Error<I2C::Error>> {
        const REG_BUFFER_LENGTH: usize = 6;
        let mut buf = [0; REG_BUFFER_LENGTH];
        let mut settings: Settings = Default::default();

        self.i2c
            .write_read(self.address, &[CONF_ADDR_START], &mut buf)
            .map_err(Error::BusError)?;

        settings.gas = self.get_gas_config()?;
        settings.gas.heater_control = HeaterControl::try_from(buf[0] & 0x7)?;
        settings.gas.nb_conversion = NBConversion::try_from(buf[1] & 0xf)?;
        settings.gas.enable_gas_measurement = ((buf[1] & 0x10) >> 4) == 0;
        settings.temperature.humidity_oversampling = Oversampling::try_from(buf[2] & 0x7)?;
        settings.temperature.temperature_oversampling =
            Oversampling::try_from((buf[4] & 0xe0) >> 5)?;
        settings.temperature.pressure_oversampling = Oversampling::try_from((buf[4] & 0x1c) >> 2)?;
        settings.temperature.iir_filter = IIRFilter::try_from((buf[5] & 0x1c) >> 2)?;

        Ok(settings)
    }

    pub fn set_power_mode<D>(
        &mut self,
        delay: &mut D,
        target_power_mode: PowerMode,
    ) -> Result<(), Error<I2C::Error>>
    where
        D: DelayMs,
    {
        loop {
            let data =
                read_byte(self.i2c, self.address, CONF_POWER_MODE_ADDR).map_err(Error::BusError)?;
            let power_mode = PowerMode::try_from(data & MODE_MASK)?;

            if power_mode != PowerMode::Sleep {
                let data = data & !MODE_MASK;
                self.i2c
                    .write(self.address, &[CONF_POWER_MODE_ADDR, data])
                    .map_err(Error::BusError)?;
                delay.delay_ms(POLL_PERIOD_MS);
            } else {
                if target_power_mode != PowerMode::Sleep {
                    let data = data & !MODE_MASK | u8::from(target_power_mode);
                    self.i2c
                        .write(self.address, &[CONF_POWER_MODE_ADDR, data])
                        .map_err(Error::BusError)?;
                }
                return Ok(());
            }
        }
    }

    pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<I2C::Error>> {
        let regs =
            read_byte(self.i2c, self.address, CONF_POWER_MODE_ADDR).map_err(Error::BusError)?;
        Ok(PowerMode::try_from(regs & MODE_MASK)?)
    }

    const OS_TO_MEAS_CYCLES: [u8; 6] = [0, 1, 2, 4, 8, 16];

    pub fn get_profile_duration(&self, settings: &Settings) -> Result<u32, Error<I2C::Error>> {
        let temperature_oversampling = settings.temperature.temperature_oversampling;
        let pressure_oversampling = settings.temperature.pressure_oversampling;
        let humidity_oversampling = settings.temperature.humidity_oversampling;

        let cycles = Self::OS_TO_MEAS_CYCLES[temperature_oversampling as usize]
            + Self::OS_TO_MEAS_CYCLES[pressure_oversampling as usize]
            + Self::OS_TO_MEAS_CYCLES[humidity_oversampling as usize];

        let temp_duration = ((cycles as u32) * 1963 + 477 * 9 + 500) / 1000 + 1;

        if settings.gas.enable_gas_measurement {
            Ok(temp_duration + settings.gas.heater_duration_ms)
        } else {
            Ok(temp_duration)
        }
    }

    fn get_calibration_data(
        i2c: &mut I2C,
        address: i2c::SevenBitAddress,
    ) -> Result<CalibrationData, Error<I2C::Error>> {
        let mut calibration_data: CalibrationData = Default::default();
        const MAX_LEN: usize =
            [COEFF_ADDR1_LEN, COEFF_ADDR2_LEN][(COEFF_ADDR1_LEN < COEFF_ADDR2_LEN) as usize];
        let mut coef = [0u8; MAX_LEN];

        i2c.write_read(address, &[COEFF_ADDR1], &mut coef[..(COEFF_ADDR1_LEN - 1)])
            .map_err(Error::BusError)?;

        calibration_data.par_t2 = ((coef[2] as i32) << 8 | coef[1] as i32) as i16;
        calibration_data.par_t3 = coef[3] as i8;
        calibration_data.par_p1 = ((coef[6] as i32) << 8 | coef[5] as i32) as u16;
        calibration_data.par_p2 = ((coef[8] as i32) << 8 | coef[7] as i32) as i16;
        calibration_data.par_p3 = coef[9] as i8;
        calibration_data.par_p4 = ((coef[12] as i32) << 8 | coef[11] as i32) as i16;
        calibration_data.par_p5 = ((coef[14] as i32) << 8 | coef[13] as i32) as i16;
        calibration_data.par_p6 = coef[16] as i8;
        calibration_data.par_p7 = coef[15] as i8;
        calibration_data.par_p8 = ((coef[20] as i32) << 8 | coef[19] as i32) as i16;
        calibration_data.par_p9 = ((coef[22] as i32) << 8 | coef[21] as i32) as i16;
        calibration_data.par_p10 = coef[23];

        i2c.write_read(address, &[COEFF_ADDR2], &mut coef[..(COEFF_ADDR2_LEN - 1)])
            .map_err(Error::BusError)?;

        calibration_data.par_t1 = ((coef[9] as i32) << 8 | coef[8] as i32) as u16;
        calibration_data.par_h1 = ((coef[2] as i32) << 4 | coef[1] as i32 & 0xf) as u16;
        calibration_data.par_h2 = ((coef[0] as i32) << 4 | coef[1] as i32 >> 4) as u16;
        calibration_data.par_h3 = coef[3] as i8;
        calibration_data.par_h4 = coef[4] as i8;
        calibration_data.par_h5 = coef[5] as i8;
        calibration_data.par_h6 = coef[6];
        calibration_data.par_h7 = coef[7] as i8;
        calibration_data.par_gh1 = coef[12] as i8;
        calibration_data.par_gh2 = ((coef[11] as i32) << 8 | coef[10] as i32) as i16;
        calibration_data.par_gh3 = coef[13] as i8;

        calibration_data.res_heat_range =
            (read_byte(i2c, address, ADDR_RES_HEAT_RANGE_ADDR).map_err(Error::BusError)? & 0x30)
                / 16;

        calibration_data.res_heat_val =
            read_byte(i2c, address, ADDR_RES_HEAT_VAL_ADDR).map_err(Error::BusError)? as i8;

        calibration_data.range_sw_err = (read_byte(i2c, address, ADDR_RANGE_SW_ERR_ADDR)
            .map_err(Error::BusError)?
            & RSERROR_MASK)
            / 16;

        Ok(calibration_data)
    }

    fn get_gas_config(&mut self) -> Result<GasSettings, Error<I2C::Error>> {
        let heater_temperature = read_byte(self.i2c, self.address, ADDR_SENS_CONF_START)
            .map_err(Error::BusError)? as u16;
        let heater_duration_ms =
            read_byte(self.i2c, self.address, ADDR_GAS_CONF_START).map_err(Error::BusError)? as u32;
        let gas_sett = GasSettings {
            heater_temperature,
            heater_duration_ms,
            ..Default::default()
        };

        Ok(gas_sett)
    }

    pub fn get_readings(&mut self) -> Result<(Readings, ReadingsCondition), Error<I2C::Error>> {
        let mut buf = [0u8; 15];
        self.i2c
            .write_read(self.address, &[FIELD0_ADDR], &mut buf)
            .map_err(Error::BusError)?;

        let new_data_mask = buf[0] & NEW_DATA_MASK;
        let status = new_data_mask | (buf[14] & (GASM_VALID_MASK | HEAT_STAB_MASK));

        if new_data_mask == 0 {
            return Ok((
                Readings {
                    status,
                    ..Default::default()
                },
                ReadingsCondition::Unchanged,
            ));
        }

        let adc_pressure =
            ((buf[2] as u32) * 4096) | ((buf[3] as u32) * 16) | ((buf[4] as u32) * 16);
        let adc_temperature =
            ((buf[5] as u32) * 4096) | ((buf[6] as u32) * 16) | ((buf[7] as u32) * 16);
        let adc_humidity = ((buf[8] as u32) * 256) | buf[9] as u32;
        let adc_gas_resistance = ((buf[13] as u32) * 4) | ((buf[14] as u32) * 64);
        let gas_range = buf[14] & GAS_RANGE_MASK;

        let (temperature, t_fine) = calc::temperature(&self.calibration_data, adc_temperature);

        Ok((
            Readings {
                status,
                temperature,
                pressure: calc::pressure(&self.calibration_data, t_fine, adc_pressure),
                humidity: calc::humidity(&self.calibration_data, t_fine, adc_humidity as u16),
                gas_resistance: calc::gas_resistance(
                    self.calibration_data.range_sw_err,
                    adc_gas_resistance as u16,
                    gas_range as usize,
                ),
            },
            ReadingsCondition::Changed,
        ))
    }
}

fn read_byte<I2C>(
    i2c: &mut I2C,
    address: i2c::SevenBitAddress,
    register: u8,
) -> Result<u8, I2C::Error>
where
    I2C: i2c::I2c,
{
    let mut buf = [0; 1];
    i2c.write_read(address, &[register], &mut buf)?;

    Ok(buf[0])
}
