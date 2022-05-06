#![no_std]
#![no_main]
#![macro_use]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

use drogue_device::bsp::boards::nrf52::adafruit_feather_nrf52840::*;
use drogue_device::drivers::ble::gatt::dfu::FirmwareGattService;
use drogue_device::drivers::ble::gatt::dfu::{FirmwareService, FirmwareServiceEvent};
use drogue_device::firmware::FirmwareManager;
use drogue_device::Board;
use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::channel::{Channel, DynamicReceiver, DynamicSender};
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_boot_nrf::updater;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::PWM0;
use embassy_nrf::pwm::{Prescaler, SimplePwm};
use embassy_nrf::Peripherals;
use heapless::Vec;
use nrf_softdevice::ble::gatt_server;
use nrf_softdevice::{
    ble::{self, peripheral, Connection},
    raw, Flash, Softdevice,
};

#[cfg(feature = "panic-probe")]
use panic_probe as _;

#[cfg(feature = "nrf-softdevice-defmt-rtt")]
use nrf_softdevice_defmt_rtt as _;

#[cfg(feature = "panic-reset")]
use panic_reset as _;

const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");
const FIRMWARE_REVISION: Option<&str> = option_env!("REVISION");

// Application must run at a lower priority than softdevice
fn config() -> Config {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config
}

#[embassy::main(config = "config()")]
async fn main(s: Spawner, p: Peripherals) {
    let board = AdafruitFeatherNrf52840::new(p);

    // Spawn the underlying softdevice task
    let sd = enable_softdevice("trainbot");
    s.spawn(softdevice_task(sd)).unwrap();

    let version = FIRMWARE_REVISION.unwrap_or(FIRMWARE_VERSION);
    defmt::info!("Running firmware version {}", version);
    defmt::info!("My address: {:?}", ble::get_address(sd));

    // Watchdog will prevent bootloader from resetting. If your application hangs for more than 5 seconds
    // (depending on bootloader config), it will enter bootloader which may swap the application back.
    s.spawn(watchdog_task()).unwrap();

    // Create a BLE GATT server and make it static
    static GATT: Forever<GattServer> = Forever::new();
    let server = GATT.put(gatt_server::register(sd).unwrap());

    // Fiwmare update service event channel and task
    static EVENTS: Channel<ThreadModeRawMutex, FirmwareServiceEvent, 10> = Channel::new();
    let dfu = FirmwareManager::new(Flash::take(sd), updater::new());
    let updater = FirmwareGattService::new(&server.firmware, dfu, version.as_bytes(), 64).unwrap();
    s.spawn(updater_task(updater, EVENTS.receiver().into()))
        .unwrap();

    // MOTOR control
    static COMMANDS: Channel<ThreadModeRawMutex, Command, 4> = Channel::new();

    let in1 = Output::new(board.d6.degrade(), Level::Low, OutputDrive::Standard);
    let in2 = Output::new(board.d5.degrade(), Level::Low, OutputDrive::Standard);
    let stdby = Output::new(board.d13.degrade(), Level::Low, OutputDrive::Standard);
    let pwm = SimplePwm::new_1ch(board.pwm0, board.a4);
    let m = Motor::new(in1, in2, pwm, stdby);

    s.spawn(motor(m, COMMANDS.receiver().into())).unwrap();

    // Starts the bluetooth advertisement and GATT server
    s.spawn(advertiser_task(
        s,
        sd,
        server,
        EVENTS.sender().into(),
        COMMANDS.sender().into(),
        "trainbot",
    ))
    .unwrap();
}

#[nrf_softdevice::gatt_server]
pub struct GattServer {
    pub firmware: FirmwareService,
    pub motor: MotorService,
}

#[nrf_softdevice::gatt_service(uuid = "00002000-b0cd-11ec-871f-d45ddf138840")]
pub struct MotorService {
    #[characteristic(uuid = "00002001-b0cd-11ec-871f-d45ddf138840", write, read)]
    control: i8,
}

#[embassy::task]
pub async fn updater_task(
    mut dfu: FirmwareGattService<'static, FirmwareManager<Flash>>,
    events: DynamicReceiver<'static, FirmwareServiceEvent>,
) {
    loop {
        let event = events.recv().await;
        if let Err(e) = dfu.handle(&event).await {
            defmt::warn!("Error applying firmware event: {:?}", e);
        }
    }
}

#[embassy::task(pool_size = "4")]
pub async fn gatt_server_task(
    conn: Connection,
    server: &'static GattServer,
    dfu: DynamicSender<'static, FirmwareServiceEvent>,
    motor: DynamicSender<'static, Command>,
) {
    let res = gatt_server::run(&conn, server, |e| match e {
        GattServerEvent::Motor(MotorServiceEvent::ControlWrite(value)) => {
            let command: Command = value.into();
            let _ = motor.try_send(command);
        }
        GattServerEvent::Firmware(e) => {
            let _ = dfu.try_send(e);
        }
    })
    .await;
    if let Err(e) = res {
        defmt::warn!("gatt_server run exited with error: {:?}", e);
    }
}

#[embassy::task]
pub async fn advertiser_task(
    spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static GattServer,
    events: DynamicSender<'static, FirmwareServiceEvent>,
    commands: DynamicSender<'static, Command>,
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
        let conn = peripheral::advertise_connectable(sd, adv, &config)
            .await
            .unwrap();

        defmt::debug!("connection established");
        if let Err(e) = spawner.spawn(gatt_server_task(
            conn,
            server,
            events.clone(),
            commands.clone(),
        )) {
            defmt::warn!("Error spawning gatt task: {:?}", e);
        }
    }
}

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

// Keeps our system alive
#[embassy::task]
async fn watchdog_task() {
    let mut handle = unsafe { embassy_nrf::wdt::WatchdogHandle::steal(0) };
    loop {
        handle.pet();
        Timer::after(Duration::from_secs(2)).await;
    }
}

pub enum Command {
    Forward(Speed),
    Stop,
    Reverse(Speed),
}

impl From<i8> for Command {
    fn from(value: i8) -> Command {
        if value == 0 {
            Command::Stop
        } else if value > 0 {
            Command::Forward(value.into())
        } else {
            Command::Reverse(value.into())
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

impl From<i8> for Speed {
    fn from(value: i8) -> Speed {
        let value = (value as i16).abs();
        if value > 110 {
            Speed::_6
        } else if value > 88 {
            Speed::_5
        } else if value > 66 {
            Speed::_4
        } else if value > 44 {
            Speed::_3
        } else if value > 22 {
            Speed::_2
        } else {
            Speed::_1
        }
    }
}

impl Into<u16> for Speed {
    fn into(self) -> u16 {
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
        self.pwm.set_duty(0, speed as u16);
        self.dir1.set_low();
        self.dir2.set_high();
    }

    pub fn reverse(&mut self, speed: Speed) {
        self.pwm.set_duty(0, speed as u16);
        self.dir1.set_high();
        self.dir2.set_low();
    }
}

#[embassy::task]
async fn motor(mut motor: Motor, commands: DynamicReceiver<'static, Command>) {
    loop {
        let c = commands.recv().await;
        match c {
            Command::Forward(speed) => {
                motor.enable();
                motor.forward(speed);
            }
            Command::Stop => {
                motor.disable();
            }
            Command::Reverse(speed) => {
                motor.enable();
                motor.reverse(speed);
            }
        }
    }
}

pub fn enable_softdevice(name: &'static str) -> &'static Softdevice {
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
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: Default::default(),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: name.as_ptr() as *const u8 as _,
            current_len: name.len() as u16,
            max_len: name.len() as u16,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };
    Softdevice::enable(&config)
}
