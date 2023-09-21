#![forbid(unsafe_code)]

use crate::CalibrationData;

const MAX_HEATER_TEMPERATURE: u16 = 400;

pub(crate) fn heater_temperature(
    calib: &CalibrationData,
    ambient_temperature: i8,
    temperature: u16,
) -> u8 {
    let temperature = temperature.min(MAX_HEATER_TEMPERATURE);

    let var1 = ambient_temperature as i32 * calib.par_gh3 as i32 / 1000 * 256;
    let var2 = (calib.par_gh1 as i32 + 784)
        * (((calib.par_gh2 as i32 + 154009) * temperature as i32 * 5 / 100 + 3276800) / 10);
    let var3 = var1 + var2 / 2;
    let var4 = var3 / (calib.res_heat_range as i32 + 4);
    let var5 = 131 * calib.res_heat_val as i32 + 65536;
    let heatr_res_x100 = (var4 / var5 - 250) * 34;
    ((heatr_res_x100 + 50) / 100) as u8
}

pub(crate) fn heater_duration(duration_ms: u32) -> u8 {
    let mut factor = 0;

    if duration_ms >= 0xfc0 {
        0xff
    } else {
        let mut dur = duration_ms;
        while dur > 0x3f {
            dur /= 4;
            factor += 1;
        }

        (dur + factor * 6) as u8
    }
}

pub(crate) fn temperature(calib: &CalibrationData, temp_adc: u32) -> (i16, i32) {
    let var1: i64 = (temp_adc as i64 >> 3) - ((calib.par_t1 as i64) << 1);
    let var2: i64 = (var1 * (calib.par_t2 as i64)) >> 11;
    let var3: i64 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    let var3: i64 = (var3 * ((calib.par_t3 as i64) << 4)) >> 14;

    let t_fine: i32 = (var2 + var3) as i32;
    let calc_temp: i16 = (((t_fine * 5) + 128) >> 8) as i16;
    (calc_temp, t_fine)
}

pub(crate) fn pressure(calib: &CalibrationData, t_fine: i32, pres_adc: u32) -> u32 {
    let mut var1: i32 = (t_fine >> 1) - 64000;
    let mut var2: i32 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * calib.par_p6 as i32) >> 2;
    var2 += (var1 * (calib.par_p5 as i32)) << 1;
    var2 = (var2 >> 2i32) + ((calib.par_p4 as i32) << 16i32);
    var1 = (((((var1 >> 2i32) * (var1 >> 2i32)) >> 13i32) * ((calib.par_p3 as i32) << 5i32))
        >> 3i32)
        + ((calib.par_p2 as i32 * var1) >> 1i32);
    var1 >>= 18i32;
    var1 = ((32768i32 + var1) * calib.par_p1 as i32) >> 15i32;
    let mut pressure_comp: i32 = 1048576u32.wrapping_sub(pres_adc) as i32;
    pressure_comp = ((pressure_comp - (var2 >> 12i32)) as u32).wrapping_mul(3125u32) as i32;
    if pressure_comp >= 0x40000000i32 {
        pressure_comp = ((pressure_comp as u32).wrapping_div(var1 as u32) << 1i32) as i32;
    } else {
        pressure_comp = ((pressure_comp << 1i32) as u32).wrapping_div(var1 as u32) as i32;
    }
    var1 = (calib.par_p9 as i32 * (((pressure_comp >> 3i32) * (pressure_comp >> 3i32)) >> 13i32))
        >> 12i32;
    var2 = ((pressure_comp >> 2i32) * calib.par_p8 as i32) >> 13i32;
    let var3: i32 = ((pressure_comp >> 8i32)
        * (pressure_comp >> 8i32)
        * (pressure_comp >> 8i32)
        * calib.par_p10 as i32)
        >> 17i32;
    pressure_comp += (var1 + var2 + var3 + ((calib.par_p7 as i32) << 7i32)) >> 4i32;
    pressure_comp as u32
}

pub(crate) fn humidity(calib: &CalibrationData, t_fine: i32, hum_adc: u16) -> u32 {
    let temp_scaled: i32 = (t_fine * 5i32 + 128i32) >> 8i32;
    let var1: i32 = hum_adc as i32
        - calib.par_h1 as i32 * 16i32
        - ((temp_scaled * calib.par_h3 as i32 / 100i32) >> 1i32);
    let var2: i32 = (calib.par_h2 as i32
        * (temp_scaled * calib.par_h4 as i32 / 100i32
            + ((temp_scaled * (temp_scaled * calib.par_h5 as i32 / 100i32)) >> 6i32) / 100i32
            + (1i32 << 14i32)))
        >> 10i32;
    let var3: i32 = var1 * var2;
    let var4: i32 = (calib.par_h6 as i32) << 7i32;
    let var4: i32 = (var4 + temp_scaled * calib.par_h7 as i32 / 100i32) >> 4i32;
    let var5: i32 = ((var3 >> 14i32) * (var3 >> 14i32)) >> 10i32;
    let var6: i32 = (var4 * var5) >> 1i32;
    let calc_hum: i32 = (((var3 + var6) >> 10i32) * 1000i32) >> 12i32;

    calc_hum.clamp(0, 100000) as u32
}

const LOOKUP_TABLE1: [u32; 16] = [
    2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2126008810, 2147483647, 2130303777,
    2147483647, 2147483647, 2143188679, 2136746228, 2147483647, 2126008810, 2147483647, 2147483647,
];

const LOOKUP_TABLE2: [u32; 16] = [
    4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064,
    16016016, 8000000, 4000000, 2000000, 1, 500000, 250000, 125000,
];

pub(crate) fn gas_resistance(range_sw_err: u8, gas_res_adc: u16, gas_range: usize) -> u32 {
    let var1 = ((1340 + 5 * range_sw_err as i64) * LOOKUP_TABLE1[gas_range] as i64) >> 16;
    let var2 = (((gas_res_adc as i64) << 15) - 16777216 + var1) as u64;
    let var3 = (LOOKUP_TABLE2[gas_range] as i64 * var1) >> 9;
    ((var3 + ((var2 as i64) >> 1)) / var2 as i64) as u32
}
