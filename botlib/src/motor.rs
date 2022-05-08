use embassy::channel::DynamicReceiver;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::PWM0;
use embassy_nrf::pwm::{Prescaler, SimplePwm};
use embassy_nrf::Peripherals;

pub enum MotorCommand {
    Forward(Speed),
    Stop,
    Reverse(Speed),
}

impl MotorCommand {
    pub fn new(value: i8) -> MotorCommand {
        if value == 0 {
            MotorCommand::Stop
        } else if value > 0 {
            MotorCommand::Forward(Speed::new(value))
        } else {
            MotorCommand::Reverse(Speed::new(value))
        }
    }
}

pub enum Speed {
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
}

impl Speed {
    pub fn new(value: i8) -> Speed {
        let value = (value as i16).abs();
        if value >= 110 {
            Speed::_6
        } else if value >= 88 {
            Speed::_5
        } else if value >= 66 {
            Speed::_4
        } else if value >= 44 {
            Speed::_3
        } else if value >= 22 {
            Speed::_2
        } else {
            Speed::_1
        }
    }

    fn duty(&self) -> u16 {
        match self {
            Self::_1 => 2500,
            Self::_2 => 2000,
            Self::_3 => 1500,
            Self::_4 => 1000,
            Self::_5 => 500,
            Self::_6 => 0,
        }
    }
}

pub struct Motor {
    dir1: Output<'static, AnyPin>,
    dir2: Output<'static, AnyPin>,
    pwm: SimplePwm<'static, PWM0>,
    standby: Output<'static, AnyPin>,
}

impl Motor {
    pub fn new(
        dir1: Output<'static, AnyPin>,
        dir2: Output<'static, AnyPin>,
        pwm: SimplePwm<'static, PWM0>,
        standby: Output<'static, AnyPin>,
    ) -> Self {
        Self {
            dir1,
            dir2,
            pwm,
            standby,
        }
    }

    pub fn enable(&mut self) {
        self.standby.set_high();
        self.pwm.set_prescaler(Prescaler::Div128);
        self.pwm.set_max_duty(2500);
        self.pwm.set_duty(0, 2500);
    }

    pub fn disable(&mut self) {
        self.standby.set_low();
    }

    pub fn forward(&mut self, speed: Speed) {
        let s = speed.duty();
        defmt::info!("Forward speed is {}", s);
        self.pwm.set_duty(0, s);
        self.dir1.set_low();
        self.dir2.set_high();
    }

    pub fn reverse(&mut self, speed: Speed) {
        let s = speed.duty();
        defmt::info!("Reverse speed is {}", s);
        self.pwm.set_duty(0, s);
        self.dir1.set_high();
        self.dir2.set_low();
    }
}

#[embassy::task]
pub async fn motor_task(mut motor: Motor, commands: DynamicReceiver<'static, MotorCommand>) {
    loop {
        let c = commands.recv().await;
        match c {
            MotorCommand::Forward(speed) => {
                motor.enable();
                motor.forward(speed);
            }
            MotorCommand::Stop => {
                motor.disable();
            }
            MotorCommand::Reverse(speed) => {
                motor.enable();
                motor.reverse(speed);
            }
        }
    }
}
