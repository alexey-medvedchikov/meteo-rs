#![cfg_attr(not(test), no_std)]
#![no_main]

use core::result::Result;
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

const SYSCLK_RATE_MHZ: u32 = 8;
const I2C_RATE_KHZ: u32 = 400;

fn run() -> Result<(), firmware::Error<hal::i2c::Error>> {
    let sysclk_rate = SYSCLK_RATE_MHZ.MHz();
    let i2c_rate = I2C_RATE_KHZ.kHz();

    let mut delay = {
        let dp = unsafe { hal::pac::Peripherals::steal() };
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(sysclk_rate).freeze();
        DelayMs(dp.TIM2.delay_ms(&clocks))
    };

    let i2c = i2c_acquire(sysclk_rate, i2c_rate);
    let mut display = firmware::display_acquire(i2c)?;
    let mut i2c = i2c_acquire(sysclk_rate, i2c_rate);
    let mut sensor = firmware::sensor_acquire(&mut i2c, &mut delay)?;

    firmware::run(&mut display, &mut sensor, &mut delay, |delay| {
        delay.0.delay_ms(1000);
    })
}

fn i2c_acquire(sysclk_rate: HertzU32, i2c_rate: HertzU32) -> hal::i2c::I2c<hal::pac::I2C1> {
    let dp = unsafe { hal::pac::Peripherals::steal() };
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(sysclk_rate).freeze();
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8;
    let sda = gpiob.pb9;
    hal::i2c::I2c::new(dp.I2C1, (scl, sda), i2c_rate, &clocks)
}

struct DelayMs(hal::timer::DelayMs<hal::pac::TIM2>);

impl firmware::DelayMs for DelayMs {
    fn delay_ms(&mut self, ms: u16) {
        self.0.delay_ms(ms as u32)
    }
}
