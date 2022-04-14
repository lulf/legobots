#![no_std]
#![no_main]
#![macro_use]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;

use drogue_device::{bsp::boards::nrf52::microbit::*, Board};

use embassy::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver},
    time::{Duration, Timer},
};
use embassy_nrf::{peripherals::PWM0, pwm::*, Peripherals};

use panic_probe as _;

#[embassy::main]
async fn main(spawner: embassy::executor::Spawner, p: Peripherals) {
    let board = Microbit::new(p);

    static COMMANDS: Channel<ThreadModeRawMutex, State, 4> = Channel::new();

    let pwm = SimplePwm::new_1ch(board.pwm0, board.p1);

    spawner.spawn(motor(pwm, COMMANDS.receiver())).unwrap();

    loop {
        COMMANDS.send(State::Left).await;
        Timer::after(Duration::from_secs(1)).await;
        COMMANDS.send(State::Right).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

pub enum State {
    Left,
    Right,
}

#[embassy::task]
async fn motor(
    mut pwm: SimplePwm<'static, PWM0>,
    commands: Receiver<'static, ThreadModeRawMutex, State, 4>,
) {
    pwm.set_prescaler(Prescaler::Div128);
    pwm.set_max_duty(2500);

    let mut position = None;
    loop {
        let c = commands.recv().await;
        match position {
            Some(State::Left) => {
                match c {
                    State::Left => {}
                    State::Right => {
                        // Move right
                        pwm.set_duty(0, 2500 - 218);
                    }
                }
            }
            Some(State::Right) => {
                match c {
                    State::Left => {
                        // Move left
                        pwm.set_duty(0, 2500 - 156);
                    }
                    State::Right => {}
                }
            }
            None => {
                // Move left first
                pwm.set_duty(0, 2500 - 156);
                match c {
                    State::Left => {}
                    State::Right => {
                        // Move right
                        pwm.set_duty(0, 2500 - 218);
                    }
                }
            }
        }
        position = Some(c);
    }
}
