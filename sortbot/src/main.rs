#![no_std]
#![no_main]

use apds9960::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_0;
    let scl = p.PIN_1;

    let irq = p.PIN_2;
    let mut irq = Input::new(irq, Pull::Up);

    let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
    let mut sensor = Apds9960::new(i2c);
    sensor.disable().unwrap();
    sensor.enable().unwrap();
    sensor.enable_proximity().unwrap();
    sensor.enable_light().unwrap();
    sensor.enable_wait().unwrap();
    // sensor.enable_proximity_interrupts().unwrap();
    sensor.set_proximity_low_threshold(5).unwrap();
    sensor.set_proximity_high_threshold(255).unwrap();
    // sensor.set_light_integration_time(72).unwrap();
    //sensor.set_color_gain(2).unwrap();
    //sensor.enable_light().unwrap();
    //sensor.enable_light_interrupts().unwrap();
    loop {
        let p = sensor.read_proximity();
        if let Ok(p) = p {
            if p > 3 {
                loop {
                    if let Ok(light) = sensor.read_light() {
                        defmt::info!("Light: {:?}", defmt::Debug2Format(&light));
                        break;
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
