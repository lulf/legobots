use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    println!("Hello, world!");
    Ok(())
}
