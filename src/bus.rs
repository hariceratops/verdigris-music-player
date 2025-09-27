use defmt::info;
use ds323x::{Ds323x, interface::I2cInterface};
use embassy_embedded_hal::shared_bus::{
    asynch::i2c::I2cDevice as AsyncI2cDevice, blocking::i2c::I2cDevice as BlockingI2cDevice,
};
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use ina219::{AsyncIna219, address::Address, calibration::UnCalibrated};

use crate::{I2c0Bus, I2c1Bus, SharedBattery};

#[derive(Debug)]
pub enum Error {
    // InitializationError,
    DisplayInitError,
    // RtcInitError,
    BatteryInitError,
}

pub(crate) type RtcType = Ds323x<
    I2cInterface<
        BlockingI2cDevice<
            'static,
            NoopRawMutex,
            I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>,
        >,
    >,
    ds323x::ic::DS3231,
>;

pub fn init_rtc(i2c_bus: &'static I2c0Bus) -> RtcType {
    let i2c_dev = BlockingI2cDevice::new(i2c_bus);
    Ds323x::new_ds3231(i2c_dev)
    // TODO Wrap in a Result
}

#[embassy_executor::task]
pub async fn rtc_job(_i2c_bus: &'static I2c0Bus) {
    loop {
        info!("Rtc job");
        Timer::after_secs(1).await
    }
}

pub(crate) type BatteryType = AsyncIna219<
    AsyncI2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>,
    UnCalibrated,
>;

pub async fn init_battery(i2c_bus: &'static I2c1Bus) -> Result<BatteryType, Error> {
    let i2c_dev = AsyncI2cDevice::new(i2c_bus);

    let ina219 = AsyncIna219::new(i2c_dev, Address::from_byte(0x43).unwrap()).await;
    if let Ok(mut ina_dev) = ina219 {
        ina_dev
            .configuration()
            .await
            .unwrap()
            .conversion_time_us()
            .unwrap();
        Ok(ina_dev)
    } else {
        Err(Error::BatteryInitError)
    }
}

#[embassy_executor::task]
pub async fn battery_job(ina: &'static SharedBattery) {
    loop {
        let mut guard = ina.lock().await;
        let shunt = guard.shunt_voltage().await.unwrap();
        let bus = guard.bus_voltage().await.unwrap();
        info!("shunt: {:?}, bus: {:?}", shunt.shunt_voltage_mv(), bus.voltage_mv());
        drop(guard);

        Timer::after_secs(1).await
    }
}
