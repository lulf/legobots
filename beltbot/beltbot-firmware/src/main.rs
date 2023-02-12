#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait)]

use botlib::motor::{MotorCommand, Motor};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, DynamicReceiver, DynamicSender};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_nrf::config::Config;
use embassy_nrf::peripherals::{P0_07, P1_08, P1_09, P0_02, PWM0};
use embassy_nrf::gpio::{Level, Output, OutputDrive, Pin};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::Peripherals;
use heapless::Vec;
use nrf_softdevice::ble::gatt_server;
use nrf_softdevice::{
    ble::{self, peripheral, Connection},
    raw, Softdevice,
};
use static_cell::StaticCell;

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

struct Board {
    d6: P0_07,
    d5: P1_08,
    d13: P1_09,
    a4: P0_02,
    pwm0: PWM0,

}

impl From<Peripherals> for Board {
    fn from(p: Peripherals) -> Self {
        Self {
            d6: p.P0_07,
            d5: p.P1_08,
            d13: p.P1_09,
            a4: p.P0_02,
            pwm0: p.PWM0,
        }
    }
}

#[embassy_executor::main]
async fn main(s: Spawner) {
    let p = embassy_nrf::init(config());
    let board: Board = p.into();

    // Spawn the underlying softdevice task
    let sd = enable_softdevice("trainbot");

    let version = FIRMWARE_REVISION.unwrap_or(FIRMWARE_VERSION);
    defmt::info!("Running firmware version {}", version);
    defmt::info!("My address: {:?}", ble::get_address(sd));

    // Watchdog will prevent bootloader from resetting. If your application hangs for more than 5 seconds
    // (depending on bootloader config), it will enter bootloader which may swap the application back.
    s.spawn(watchdog_task()).unwrap();

    // Create a BLE GATT server and make it static
    static GATT: StaticCell<GattServer> = StaticCell::new();
    let server = GATT.init(GattServer::new(sd).unwrap());

    s.spawn(softdevice_task(sd)).unwrap();

    // Fiwmare update service event channel and task
    //static EVENTS: Channel<ThreadModeRawMutex, FirmwareServiceEvent, 10> = Channel::new();
    //let dfu = FirmwareManager::new(Flash::take(sd), updater::new());
    //let updater = FirmwareGattService::new(&server.firmware, dfu, version.as_bytes(), 32).unwrap();
    //s.spawn(updater_task(updater, EVENTS.receiver().into()))
    //    .unwrap();

    // MOTOR control
    static COMMANDS: Channel<ThreadModeRawMutex, MotorCommand, 4> = Channel::new();

    let in1 = Output::new(board.d6.degrade(), Level::Low, OutputDrive::Standard);
    let in2 = Output::new(board.d5.degrade(), Level::Low, OutputDrive::Standard);
    let stdby = Output::new(board.d13.degrade(), Level::Low, OutputDrive::Standard);
    let pwm = SimplePwm::new_1ch(board.pwm0, board.a4);
    let m = Motor::new(in1, in2, pwm, stdby);

    s.spawn(motor_task(m, COMMANDS.receiver().into())).unwrap();

    // Starts the bluetooth advertisement and GATT server
    s.spawn(advertiser_task(
        s,
        sd,
        server,
        //EVENTS.sender().into(),
        COMMANDS.sender().into(),
        "trainbot",
    ))
    .unwrap();
}

#[nrf_softdevice::gatt_server]
pub struct GattServer {
//    pub firmware: FirmwareService,
    pub motor: MotorService,
}

#[nrf_softdevice::gatt_service(uuid = "00002000-b0cd-11ec-871f-d45ddf138840")]
pub struct MotorService {
    #[characteristic(uuid = "00002001-b0cd-11ec-871f-d45ddf138840", write, read)]
    control: i8,
}

//#[embassy_executor::task]
//pub async fn updater_task(
//    mut dfu: FirmwareGattService<'static, FirmwareManager<Flash>>,
//    events: DynamicReceiver<'static, FirmwareServiceEvent>,
//) {
//    loop {
//        let event = events.recv().await;
//        if let Err(e) = dfu.handle(&event).await {
//            defmt::warn!("Error applying firmware event: {:?}", e);
//        }
//    }
//}

#[embassy_executor::task]
pub async fn motor_task(mut motor: Motor, commands: DynamicReceiver<'static, MotorCommand>) {
    motor.run(commands).await;
}

#[embassy_executor::task(pool_size = "4")]
pub async fn gatt_server_task(
    conn: Connection,
    server: &'static GattServer,
//    dfu: DynamicSender<'static, FirmwareServiceEvent>,
    motor: DynamicSender<'static, MotorCommand>,
) {
    let res = gatt_server::run(&conn, server, |e| match e {
        GattServerEvent::Motor(MotorServiceEvent::ControlWrite(value)) => {
            let command: MotorCommand = MotorCommand::new(value);
            let _ = motor.try_send(command);
        }
     //   GattServerEvent::Firmware(e) => {
     //       let _ = dfu.try_send(e);
     //   }
    })
    .await;
    if let Err(e) = res {
        defmt::warn!("gatt_server run exited with error: {:?}", e);
    }
}

#[embassy_executor::task]
pub async fn advertiser_task(
    spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static GattServer,
 //   events: DynamicSender<'static, FirmwareServiceEvent>,
    commands: DynamicSender<'static, MotorCommand>,
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
            //events.clone(),
            commands.clone(),
        )) {
            defmt::warn!("Error spawning gatt task: {:?}", e);
        }
        commands.send(MotorCommand::Stop).await;
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

// Keeps our system alive
#[embassy_executor::task]
async fn watchdog_task() {
    let mut handle = unsafe { embassy_nrf::wdt::WatchdogHandle::steal(0) };
    loop {
        handle.pet();
        Timer::after(Duration::from_secs(2)).await;
    }
}

pub fn enable_softdevice(name: &'static str) -> &'static mut Softdevice {
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
