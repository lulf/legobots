#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::{P0_05, TWISPI0, TWISPI1};
use embassy_nrf::peripherals::{PWM0, PWM1};
use embassy_nrf::pwm::{Prescaler, SimplePwm};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::twim::Twim;
use embassy_nrf::{bind_interrupts, peripherals, saadc, spim, twim};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, DynamicReceiver, DynamicSender};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use nrf_softdevice::{
    ble::{gatt_server, peripheral, Connection},
    raw, Softdevice,
};
use static_cell::StaticCell;

// Application must run at a lower priority than softdevice
fn config() -> Config {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config
}

#[embassy_executor::main]
async fn main(s: Spawner) {
    let p = embassy_nrf::init(config());
    let sd = enable_softdevice("Carbot Car");

    static GATT: StaticCell<CarBotServer> = StaticCell::new();
    let server = GATT.init(CarBotServer::new(sd).unwrap());

    static MOTOR: Channel<ThreadModeRawMutex, MotorCommand, 4> = Channel::new();
    static SERVO: Channel<ThreadModeRawMutex, ServoCommand, 4> = Channel::new();

    s.spawn(softdevice_task(sd)).unwrap();

    let stdby = Output::new(p.P1_08.degrade(), Level::Low, OutputDrive::Standard);

    let ain1 = Output::new(p.P0_06.degrade(), Level::Low, OutputDrive::Standard);
    let ain2 = Output::new(p.P0_27.degrade(), Level::Low, OutputDrive::Standard);
    let pwm0 = SimplePwm::new_1ch(p.PWM0, p.P0_04);
    let m1 = Motor::new(ain1, ain2, pwm0);
    let ctrl = MotorController::new(m1, stdby);

    s.spawn(motor_task(ctrl, MOTOR.receiver().into())).unwrap();

    let pwm1 = SimplePwm::new_1ch(p.PWM1, p.P0_30);
    let servo = Servo::new(pwm1);
    let ctrl = ServoController::new(servo);
    s.spawn(servo_task(ctrl, SERVO.receiver().into())).unwrap();

    // Starts the bluetooth advertisement and GATT server
    s.spawn(advertiser_task(
        s,
        sd,
        server,
        MOTOR.sender().into(),
        SERVO.sender().into(),
        "carbot",
    ))
    .unwrap();

    // Power on self test.
    Timer::after(Duration::from_secs(1)).await;
    // Run motor forward at max speed for 10 secs
    MOTOR.send(MotorCommand::Forward(Speed::_6)).await;
    Timer::after(Duration::from_secs(5)).await;
    //
    // Run motor backward at max speed for 10 secs
    MOTOR.send(MotorCommand::Reverse(Speed::_6)).await;
    Timer::after(Duration::from_secs(5)).await;

    MOTOR.send(MotorCommand::Stop).await;
    Timer::after(Duration::from_secs(5)).await;
    //
    // Swing left
    //
    SERVO.send(ServoCommand::StepLeft).await;
    Timer::after(Duration::from_secs(5)).await;
    //
    // Swing center
    SERVO.send(ServoCommand::Center).await;
    Timer::after(Duration::from_secs(5)).await;
    //
    // Swing right
    SERVO.send(ServoCommand::StepRight).await;
    Timer::after(Duration::from_secs(5)).await;

    SERVO.send(ServoCommand::StepLeft).await;
    Timer::after(Duration::from_secs(5)).await;

    SERVO.send(ServoCommand::Center).await;
    Timer::after(Duration::from_secs(5)).await;
}

#[nrf_softdevice::gatt_server]
pub struct CarBotServer {
    pub service: CarBotService,
}

#[nrf_softdevice::gatt_service(uuid = "00002000-b0cd-11ec-871f-d45ddf138840")]
pub struct CarBotService {
    #[characteristic(uuid = "00002001-b0cd-11ec-871f-d45ddf138840", write, read)]
    motor_control: i8,

    #[characteristic(uuid = "00002002-b0cd-11ec-871f-d45ddf138840", write, read)]
    servo_control: i8,
}

#[embassy_executor::task(pool_size = "2")]
pub async fn gatt_server_task(
    conn: Connection,
    server: &'static CarBotServer,
    motor: DynamicSender<'static, MotorCommand>,
    servo: DynamicSender<'static, ServoCommand>,
) {
    let _res = gatt_server::run(&conn, server, |e| match e {
        CarBotServerEvent::Service(CarBotServiceEvent::MotorControlWrite(value)) => {
            let command: MotorCommand = MotorCommand::new(value);
            let _ = motor.try_send(command);
        }
        CarBotServerEvent::Service(CarBotServiceEvent::ServoControlWrite(value)) => {
            let command: ServoCommand = ServoCommand::new(value);
            let _ = servo.try_send(command);
        }
    })
    .await;
    defmt::warn!("connection closed");
}

#[embassy_executor::task]
pub async fn advertiser_task(
    spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static CarBotServer,
    motor: DynamicSender<'static, MotorCommand>,
    servo: DynamicSender<'static, ServoCommand>,
    name: &'static str,
) {
    let mut adv_data: Vec<u8, 31> = Vec::new();
    #[rustfmt::skip]
        adv_data.extend_from_slice(&[
            0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
            0x03, 0x03, 0x00, 0x61,
            (1 + name.len() as u8), 0x09]).unwrap();

    adv_data.extend_from_slice(name.as_bytes()).ok().unwrap();

    #[rustfmt::skip]
        let scan_data = &[
            0x03, 0x03, 0xA, 0x18,
        ];

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &adv_data[..],
            scan_data,
        };
        defmt::debug!("Advertising");
        let conn = peripheral::advertise_connectable(sd, adv, &config).await.unwrap();

        defmt::debug!("connection established");
        if let Err(e) = spawner.spawn(gatt_server_task(conn, server, motor.clone(), servo.clone())) {
            defmt::warn!("Error spawning gatt task: {:?}", e);
        }
        motor.send(MotorCommand::Stop).await;
        servo.send(ServoCommand::Center).await;
    }
}

#[embassy_executor::task]
pub async fn motor_task(mut motor: MotorController, m1: DynamicReceiver<'static, MotorCommand>) {
    motor.run(m1).await;
}

#[embassy_executor::task]
pub async fn servo_task(mut servo: ServoController, m1: DynamicReceiver<'static, ServoCommand>) {
    servo.run(m1).await;
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

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
    dir1: Output<'static>,
    dir2: Output<'static>,
    pwm: SimplePwm<'static, T>,
}

impl<T: embassy_nrf::pwm::Instance> Motor<T> {
    pub fn new(dir1: Output<'static>, dir2: Output<'static>, pwm: SimplePwm<'static, T>) -> Self {
        Self { dir1, dir2, pwm }
    }

    pub fn enable(&mut self) {
        self.pwm.enable();
        self.pwm.set_prescaler(Prescaler::Div128);
        self.pwm.set_max_duty(2500);
        self.pwm.set_duty(0, 2500);
    }

    pub fn disable(&mut self) {
        self.pwm.disable();
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
    standby: Output<'static>,
}

impl MotorController {
    pub fn new(m1: Motor<PWM0>, standby: Output<'static>) -> Self {
        Self { m1, standby }
    }

    pub fn enable(&mut self) {
        self.standby.set_high();
    }

    pub fn disable(&mut self) {
        self.standby.set_low();
    }

    pub async fn run(&mut self, m1: DynamicReceiver<'static, MotorCommand>) {
        let mut m1on = false;
        loop {
            match m1.receive().await {
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
            }
            if m1on {
                self.enable();
            } else {
                self.disable();
            }
        }
    }
}

pub enum ServoCommand {
    StepLeft,
    Center,
    StepRight,
}

impl ServoCommand {
    pub fn new(value: i8) -> Self {
        if value == 0 {
            ServoCommand::Center
        } else if value > 0 {
            ServoCommand::StepLeft
        } else {
            ServoCommand::StepRight
        }
    }
}

pub struct Servo<T: embassy_nrf::pwm::Instance> {
    pwm: SimplePwm<'static, T>,
}

const SERVO_STEP: u16 = 1000;
impl<T: embassy_nrf::pwm::Instance> Servo<T> {
    pub fn new(pwm: SimplePwm<'static, T>) -> Self {
        Self { pwm }
    }

    pub fn enable(&mut self) {
        self.pwm.enable();
        self.pwm.set_prescaler(Prescaler::Div128);
        self.pwm.set_max_duty(2500);
        self.pwm.set_duty(0, 2500 / 2);
    }

    pub fn disable(&mut self) {
        self.pwm.disable();
    }

    pub fn left(&mut self) {
        let mut duty = self.pwm.duty(0);
        duty = if duty < SERVO_STEP { 0 } else { duty - SERVO_STEP };
        defmt::info!("Servo duty is {}", duty);
        self.pwm.set_duty(0, duty);
    }

    pub fn right(&mut self) {
        let mut duty = self.pwm.duty(0);
        duty = core::cmp::min(2500, duty + SERVO_STEP);
        defmt::info!("Servo duty is {}", duty);
        self.pwm.set_duty(0, duty);
    }

    pub fn center(&mut self) {
        defmt::info!("Center");
        self.pwm.set_duty(0, 2500 / 2);
    }
}

pub struct ServoController {
    servo: Servo<PWM1>,
}

impl ServoController {
    pub fn new(servo: Servo<PWM1>) -> Self {
        Self { servo }
    }

    pub async fn run(&mut self, m1: DynamicReceiver<'static, ServoCommand>) {
        self.servo.enable();
        loop {
            match m1.receive().await {
                ServoCommand::StepLeft => {
                    self.servo.left();
                }
                ServoCommand::Center => {
                    self.servo.center();
                }
                ServoCommand::StepRight => {
                    self.servo.right();
                }
            }
        }
    }
}

fn enable_softdevice(name: &'static str) -> &'static mut Softdevice {
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 4,
            rc_temp_ctiv: 2,
            accuracy: 7,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 2,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 1,
            central_sec_count: 1,
            _bitfield_1: Default::default(),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: name.as_ptr() as *const u8 as _,
            current_len: name.len() as u16,
            max_len: name.len() as u16,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };
    Softdevice::enable(&config)
}
