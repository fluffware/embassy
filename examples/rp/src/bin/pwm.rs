#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_rp::pwm::Slice;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const TOP: u16 = 65534;
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Set up a PWM slice. Every slice has two channels that can be
    // used by a specific pin.
    // Set wrap counting at 65534 and divide clock with 10+3/16
    // This gives a PWM frequency of 125e6/(10+3/16)/65534 Hz = 187.2 Hz
    let (ch0, ch1) = p.PWM_SLICE0.config().top(65534).div(10, 3).split();

    // Start with an output an 50% duty cycle on pin 0
    let pwm0 = ch0.compare(TOP / 2).pin(p.PIN_0).enable();

    // Start with an output an 25% duty cycle on pin 1
    let _pwm1 = ch1.compare(TOP / 4).pin(p.PIN_1).enable();

    // Increase the duty cycle of pin 0 every 1/10s and wrap when at TOP 
    let mut level0 = 0;
    loop {
        Timer::after(Duration::from_millis(100)).await;
        pwm0.compare(level0);
        if level0 <= TOP - 256 {
            level0 += 256;
        } else {
            level0 = 0;
        }
    }
}
