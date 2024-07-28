#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(abi_avr_interrupt)]

use atmega_hal as hal;
use atmega_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use avr_device as device;
use core::result::Result;
use panic_halt as _;

type Clock = hal::clock::MHz8;

const I2C_SPEED: u32 = 400_000;

#[allow(non_snake_case)]
#[avr_device::interrupt(atmega1284p)]
fn TIMER1_COMPA() {}

#[device::entry]
fn main() -> ! {
    loop {
        unsafe { run().unwrap_unchecked() };
    }
}

fn run() -> Result<(), firmware::Error<hal::i2c::Error>> {
    let dp = unsafe { hal::Peripherals::steal() };
    configure_power(&dp);

    let mut display = firmware::display_acquire(i2c_acquire::<Clock>())?;
    let mut delay = DelayMs(hal::delay::Delay::<Clock>::new());
    let mut i2c = i2c_acquire::<Clock>();
    let mut sensor = firmware::sensor_acquire(&mut i2c, &mut delay)?;

    timer1_init(&dp.TC1);

    unsafe { avr_device::interrupt::enable() };
    firmware::run(&mut display, &mut sensor, &mut delay, |_| {
        dp.CPU.smcr.write(|w| {
            w.sm().idle();
            w.se().set_bit();
            w
        });

        avr_device::asm::sleep();
    })
}

fn timer1_init(timer: &hal::pac::TC1) {
    timer.tccr1b.write(|w| {
        w.cs1().prescale_1024();
        w.wgm1().bits(0b01);
        w
    });

    timer.ocr1a.write(|w| w.bits(7812)); // ~1.000064s at 8Mhz
    timer.timsk1.write(|w| w.ocie1a().set_bit());
}

fn configure_power(dp: &hal::Peripherals) {
    dp.CPU.prr0.write(|w| {
        w.pradc().set_bit();
        w.prspi().set_bit();
        w.prtim0().set_bit();
        w.prtim2().set_bit();
        w.prusart0().set_bit();
        w.prusart1().set_bit();
        w
    });

    dp.CPU.prr1.write(|w| {
        w.prtim3().set_bit();
        w
    });
}

fn i2c_acquire<CLOCK>() -> hal::i2c::I2c<CLOCK>
where
    CLOCK: hal::clock::Clock,
{
    let dp = unsafe { hal::Peripherals::steal() };
    let pins = hal::pins!(dp);
    hal::i2c::I2c::<CLOCK>::new(
        dp.TWI,
        pins.pc1.into_pull_up_input(),
        pins.pc0.into_pull_up_input(),
        I2C_SPEED,
    )
}

struct DelayMs(hal::delay::Delay<Clock>);

impl firmware::DelayMs for DelayMs {
    fn delay_ms(&mut self, ms: u16) {
        self.0.delay_ms(ms);
    }
}
