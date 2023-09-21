#![cfg_attr(not(test), no_std)]
#![no_main]

use cortex_m_rt::entry;
use fugit::HertzU32;
use hal::prelude::*;
use panic_halt as _;
use stm32f4xx_hal as hal;

#[entry]
fn main() -> ! {
    loop {
        let _ = run();
    }
}

fn run() -> Result<(), firmware::Error<hal::i2c::Error, hal::i2c::Error>> {
    let sysclk_rate = 8.MHz();
    let i2c_rate = 400.kHz();

    let mut delay = {
        let dp = unsafe { hal::pac::Peripherals::steal() };
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(sysclk_rate).freeze();
        dp.TIM2.delay_ms(&clocks)
    };

    let i2c = i2c_acquire(sysclk_rate, i2c_rate);
    let mut display = firmware::display_acquire(i2c)?;
    let mut i2c = i2c_acquire(sysclk_rate, i2c_rate);
    let mut sensor = firmware::sensor_acquire(&mut i2c, &mut delay)?;

    firmware::run(&mut display, &mut sensor, &mut delay, |delay| {
        delay.delay_ms(1000u16);
    })
}

fn i2c_acquire(sysclk_rate: HertzU32, i2c_rate: HertzU32) -> hal::i2c::I2c<hal::pac::I2C1> {
    let dp = unsafe { hal::pac::Peripherals::steal() };
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(sysclk_rate).freeze();
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8;
    let sda = gpiob.pb9;
    let i2c = hal::i2c::I2c::new(dp.I2C1, (scl, sda), i2c_rate, &clocks);
    i2c
}
