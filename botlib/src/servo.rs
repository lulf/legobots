pub struct Servo {
    pwm: SimplePwm<'static, PWM1>,
}}

pub type ServoId = u8;

pub enum ServoCommand {
    StepLeft,
    StepRight,
}

pub async fn servo_task(
    mut servo: Servo,
    commands: DynamicReceiver<'static, ServoCommand>,
) {
    pwm.set_prescaler(Prescaler::Div128);
    pwm.set_max_duty(2500);

    const LEFT: u16 = 2500 - 160;
    const RIGHT: u16 = 2500 - 210;
    const DELAY: Duration = Duration::from_millis(200);
    loop {
        let c = commands.recv().await;
        match c {
            ServoCommand::SwingLeft => {
                // Move right
                pwm.set_duty(0, RIGHT);
                Timer::after(DELAY * 2).await;

                // Move left
                pwm.set_duty(0, LEFT);
                Timer::after(DELAY).await;
            }
            ServoCommand::SwingRight => {
                // Move left
                pwm.set_duty(0, LEFT);
                Timer::after(DELAY * 2).await;

                // Move right
                pwm.set_duty(0, RIGHT);
                Timer::after(DELAY).await;
            }
        }
    }
}
