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
        /*
        COMMANDS.send(Command::SwingLeft).await;
        Timer::after(Duration::from_secs(2)).await;
        COMMANDS.send(Command::SwingRight).await;
        Timer::after(Duration::from_secs(2)).await;
        */
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
