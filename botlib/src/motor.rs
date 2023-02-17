use embassy_nrf::gpio::{AnyPin, Output};
use embassy_nrf::peripherals::{PWM1, PWM0};
use embassy_nrf::pwm::{Prescaler, SimplePwm};
use embassy_sync::channel::DynamicReceiver;
use embassy_futures::select::{select, Either};

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

pub struct Motor<T: embassy_nrf::pwm::Instance> {
    dir1: Output<'static, AnyPin>,
    dir2: Output<'static, AnyPin>,
    pwm: SimplePwm<'static, T>,
}

impl<T: embassy_nrf::pwm::Instance> Motor<T> {
    pub fn new(dir1: Output<'static, AnyPin>, dir2: Output<'static, AnyPin>, pwm: SimplePwm<'static, T>) -> Self {
        Self { dir1, dir2, pwm }
    }

    pub fn enable(&mut self) {
        self.pwm.set_prescaler(Prescaler::Div128);
        self.pwm.set_max_duty(2500);
        self.pwm.set_duty(0, 2500);
    }

    pub fn disable(&mut self) {
        self.dir1.set_low();
        self.dir2.set_low();
    }

    pub fn forward(&mut self, speed: Speed) {
        let s = speed.duty();
        defmt::info!("Forward speed duty is {}", s);
        self.pwm.set_duty(0, s);
        self.dir1.set_low();
        self.dir2.set_high();
    }

    pub fn reverse(&mut self, speed: Speed) {
        let s = speed.duty();
        defmt::info!("Reverse speed duty is {}", s);
        self.pwm.set_duty(0, s);
        self.dir1.set_high();
        self.dir2.set_low();
    }
}

pub struct MotorController {
    m1: Motor<PWM0>,
    m2: Motor<PWM1>,
    standby: Output<'static, AnyPin>,
}

impl MotorController {
    pub fn new(m1: Motor<PWM0>, m2: Motor<PWM1>, standby: Output<'static, AnyPin>) -> Self {
        Self { m1, m2, standby }
    }

    pub fn enable(&mut self) {
        self.standby.set_high();
    }

    pub fn disable(&mut self) {
        self.standby.set_low();
    }

    pub async fn run(
        &mut self,
        m1: DynamicReceiver<'static, MotorCommand>,
        m2: DynamicReceiver<'static, MotorCommand>,
    ) {
        let mut m1on = false;
        let mut m2on = false;
        loop {
            match select(m1.recv(), m2.recv()).await {
                Either::First(c) => match c {
                    MotorCommand::Forward(speed) => {
                        self.m1.enable();
                        self.m1.forward(speed);
                        m1on = true;
                    }
                    MotorCommand::Stop => {
                        self.m1.disable();
                        m1on = false;
                    }
                    MotorCommand::Reverse(speed) => {
                        self.m1.enable();
                        self.m1.reverse(speed);
                        m1on = true;
                    }
                },
                Either::Second(c) => match c {
                    MotorCommand::Forward(speed) => {
                        self.m2.enable();
                        self.m2.forward(speed);
                        m2on = true;
                    }
                    MotorCommand::Stop => {
                        self.m2.disable();
                        m2on = false;
                    }
                    MotorCommand::Reverse(speed) => {
                        self.m2.enable();
                        self.m2.reverse(speed);
                        m2on = true;
                    }
                },
            }
            if m1on || m2on {
                self.enable();
            } else {
                self.disable();
            }
        }
    }
}
