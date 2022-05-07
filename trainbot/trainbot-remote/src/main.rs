#![no_std]
#![no_main]
#![macro_use]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

use drogue_device::bsp::boards::nrf52::microbit::*;
use drogue_device::Board;
use embassy::executor::Spawner;
use embassy::util::{select, Either};
use embassy_nrf::config::Config;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::Peripherals;
use nrf_softdevice::{
    ble::{central, gatt_client, Address, AddressType},
    raw, Softdevice,
};

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

    defmt::info!("Hello");

    // Spawn the underlying softdevice task
    let sd = enable_softdevice("trainbot remote");
    s.spawn(softdevice_task(sd)).unwrap();

    // Spawn control client
    s.spawn(remote_task(sd, board.btn_a, board.btn_b)).unwrap();
}

#[nrf_softdevice::gatt_client(uuid = "00002000-b0cd-11ec-871f-d45ddf138840")]
pub struct MotorServiceClient {
    #[characteristic(uuid = "00002001-b0cd-11ec-871f-d45ddf138840", write, read)]
    control: i8,
}

#[embassy::task]
async fn remote_task(sd: &'static Softdevice, mut a: PinButtonA, mut b: PinButtonB) {
    let addrs = &[&Address::new(
        AddressType::RandomStatic,
        [0x80, 0x01, 0x92, 0xcb, 0x18, 0xed],
    )];
    let mut config = central::ConnectConfig::default();
    config.scan_config.whitelist = Some(addrs);
    config.conn_params.min_conn_interval = 40;
    config.conn_params.max_conn_interval = 80;

    loop {
        defmt::info!("Connecting to peripheral");
        match central::connect(sd, &config).await {
            Ok(conn) => {
                defmt::info!("connected");
                let client: MotorServiceClient = gatt_client::discover(&conn).await.unwrap();

                // Read current state
                let value: i8 = client.control_read().await.unwrap_or(0);
                defmt::info!("read control value: {}", value);

                const MAX: i16 = i8::MAX as i16;
                const MIN: i16 = i8::MIN as i16;
                let mut value = 0;
                const STEP: i16 = 22;
                loop {
                    defmt::info!("Waiting for button press");
                    match select(a.wait_for_any_edge(), b.wait_for_any_edge()).await {
                        Either::First(_) => {
                            if a.is_low() {
                                defmt::info!("BUTTON A EVENT");
                                if value as i16 + STEP <= MAX {
                                    value += STEP as i8;
                                }
                            }
                        }
                        Either::Second(_) => {
                            if b.is_low() {
                                defmt::info!("BUTTON B EVENT");
                                if value as i16 - STEP >= MIN {
                                    value -= STEP as i8;
                                }
                            }
                        }
                    }
                    defmt::info!("Writing new value {}", value);
                    if let Err(e) = client.control_write(value).await {
                        defmt::warn!("Error writing control value: {:?}", e);
                        break;
                    }
                }
            }
            Err(e) => {
                defmt::warn!("Error connecting: {:?}", e);
            }
        }
    }
}

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
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
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 32 }),
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
