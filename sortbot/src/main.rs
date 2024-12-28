#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_time::{Duration, Timer};

mod apds9960;
use apds9960::*;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_0;
    let scl = p.PIN_1;

    let irq = p.PIN_2;
    let mut irq = Input::new(irq, Pull::Up);

    let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
    let mut sensor = Apds9960::new(DeviceInterface { bus: i2c });

    unwrap!(sensor.enable().write(|reg| {
        reg.set_pon(false);
        reg.set_pen(false);
        reg.set_pien(false);
        reg.set_aen(false);
    }));
    Timer::after(Duration::from_millis(100)).await;
    unwrap!(sensor.pers().write(|reg| {
        reg.set_ppers(2);
    }));
    unwrap!(sensor.pilt().write(|reg| {
        reg.set_value(0);
    }));
    unwrap!(sensor.piht().write(|reg| {
        reg.set_value(4);
    }));
    unwrap!(sensor.atime().write(|reg| {
        reg.set_value(0x00);
    }));
    unwrap!(sensor.control().write(|reg| {
        reg.set_again(2);
    }));
    unwrap!(sensor.enable().write(|reg| {
        reg.set_pon(true);
        reg.set_pen(true);
        reg.set_pien(true);
        reg.set_aen(true);
    }));
    Timer::after(Duration::from_millis(100)).await;
    loop {
        irq.wait_for_low().await;
        let status = unwrap!(sensor.status().read());
        if status.pvalid() {
            let data = unwrap!(sensor.pdata().read());
            if data.value() >= 5 && status.avalid() {
                info!("proximity! {}", data.value());
                let color = unwrap!(sensor.rgbc().read());
                let red = (255.0 * color.red() as f32 / color.clear() as f32) as u8;
                let green = (255.0 * color.green() as f32 / color.clear() as f32) as u8;
                let blue = (255.0 * color.blue() as f32 / color.clear() as f32) as u8;
                info!(
                    "color! clear = {}, r = {}, g = {}, b = {}",
                    color.clear(),
                    red,
                    green,
                    blue
                );
                let max = red.max(green).max(blue);
                if max == red {
                    info!("RED");
                } else if max == green {
                    info!("GREEN");
                } else if max == blue {
                    info!("BLUE");
                }
            }
        }
        unwrap!(sensor.piclear().write(|reg| {
            reg.set_value(0xFF);
        }));
    }
}

enum Color {
    Red,
    Green,
    Blue,
}
//const BRICK_COLORS: &[(Color, (u8, u8, u8), ()] =
