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
use futures::{
    future::{select, Either},
    pin_mut,
};

use panic_probe as _;

#[embassy::main]
async fn main(spawner: embassy::executor::Spawner, p: Peripherals) {
    let board = Microbit::new(p);

    static COMMANDS: Channel<ThreadModeRawMutex, Command, 4> = Channel::new();

    let pwm = SimplePwm::new_1ch(board.pwm0, board.p1);

    spawner.spawn(motor(pwm, COMMANDS.receiver())).unwrap();

    let mut a = board.btn_a;
    let mut b = board.btn_b;
    loop {
        let af = a.wait_for_rising_edge();
        let bf = b.wait_for_rising_edge();
        pin_mut!(af);
        pin_mut!(bf);
        match select(af, bf).await {
            Either::Left(_) => {
                COMMANDS.send(Command::SwingLeft).await;
            }
            Either::Right(_) => {
                COMMANDS.send(Command::SwingRight).await;
            }
        }
    }
}

pub enum Command {
    SwingLeft,
    SwingRight,
}

#[embassy::task]
async fn motor(
    mut pwm: SimplePwm<'static, PWM0>,
    commands: Receiver<'static, ThreadModeRawMutex, Command, 4>,
) {
    pwm.set_prescaler(Prescaler::Div128);
    pwm.set_max_duty(2500);

    loop {
        let c = commands.recv().await;
        match c {
            Command::SwingLeft => {
                // Move right
                pwm.set_duty(0, 2500 - 302);
                Timer::after(Duration::from_millis(500)).await;

                // Move left
                pwm.set_duty(0, 2500 - 125);
                Timer::after(Duration::from_millis(500)).await;
            }
            Command::SwingRight => {
                // Move left
                pwm.set_duty(0, 2500 - 125);
                Timer::after(Duration::from_millis(500)).await;

                // Move right
                pwm.set_duty(0, 2500 - 302);
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }
}
