#![no_std]
#![no_main]
#![macro_use]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

use drogue_device::bsp::boards::nrf52::microbit::*;
use drogue_device::Board;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_nrf::config::Config;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::Peripherals;
use nrf_softdevice::{
    ble::{central, AddressType, Address, gatt_client},
    raw, Softdevice,
};
use embassy::util::{select, Either};

#[cfg(feature = "panic-probe")]
use panic_probe as _;

#[cfg(feature = "nrf-softdevice-defmt-rtt")]
use nrf_softdevice_defmt_rtt as _;

#[cfg(feature = "panic-reset")]
use panic_reset as _;

// Application must run at a lower priority than softdevice
fn config() -> Config {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config
}

#[embassy::main(config = "config()")]
async fn main(s: Spawner, p: Peripherals) {
    let board = Microbit::new(p);

    // Spawn the underlying softdevice task
    let sd = enable_softdevice("trainbot remote");
    s.spawn(softdevice_task(sd)).unwrap();

    // Watchdog will prevent bootloader from resetting. If your application hangs for more than 5 seconds
    // (depending on bootloader config), it will enter bootloader which may swap the application back.
    s.spawn(watchdog_task()).unwrap();

    // Spawn control client
    s.spawn(remote_task(sd, board.btn_a, board.btn_b)).unwrap();
}

#[nrf_softdevice::gatt_client(uuid = "00002000-b0cd-11ec-871f-d45ddf138840")]
pub struct MotorServiceClient {
    #[characteristic(uuid = "00002001-b0cd-11ec-871f-d45ddf138840", write, read)]
    control: i8,
}

#[embassy::task]
async fn remote_task(
    sd: &'static Softdevice,
    mut a: PinButtonA,
    mut b: PinButtonB,
) {
    let addrs = &[&Address::new(
        AddressType::RandomStatic,
        [0x06, 0x6b, 0x71, 0x2c, 0xf5, 0xc0],
    )];
    let mut config = central::ConnectConfig::default();
    config.scan_config.whitelist = Some(addrs);
    if let Ok(conn) = central::connect(sd, &config).await {
        defmt::info!("connected");
        let client: MotorServiceClient = gatt_client::discover(&conn).await.unwrap();

        // Read current state
        let mut value = client.control_read().await.unwrap_or(0);
        defmt::info!("read control value: {}", value);

        const STEP: i8 = 22;
        loop {
            match select(a.wait_for_any_edge(), b.wait_for_any_edge()).await {
                Either::First(_) => {
                    if a.is_low() {
                        if value < 0 {
                            value += STEP;
                        } else if value <= i8::MAX {
                            value += core::cmp::min(STEP, i8::MAX - value);
                        }
                    }
                }
                Either::Second(_) => {
                    if b.is_low() {
                        if value > 0 {
                            value -= STEP;
                        } else if value >= i8::MIN {
                            value -= core::cmp::min(STEP, value - i8::MIN);
                        }
                    }
                }
            }
            let _ = client.control_write(value).await;
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
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
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
