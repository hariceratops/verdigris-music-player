use core::cell::RefCell;

use defmt::info;
use ds323x::Ds323x;
use embassy_embedded_hal::shared_bus::{
    asynch::i2c::I2cDevice as AsyncI2cDevice,
    blocking::{i2c::I2cDevice as BlockingI2cDevice, spi::SpiDeviceWithConfig},
};
use embassy_rp::{
    Peri, bind_interrupts,
    gpio::{Level, Output},
    i2c::{self, Async, I2c, InterruptHandler as I2CInterruptHandler},
    peripherals::{I2C0, I2C1, PIN_6, PIN_7, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27, SPI0},
    spi::{self, Spi},
};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::NoopRawMutex},
    mutex::Mutex as AsyncMutex,
};
use embassy_time::{Delay, Timer};
use mipidsi::{
    Builder,
    interface::SpiInterface,
    models::ILI9341Rgb565,
    options::{Orientation, Rotation},
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use ina219::{AsyncIna219, address::Address, calibration::UnCalibrated};

use crate::rtc::RtcTimeSource;

// Busses
pub(crate) type I2c0Bus = Mutex<NoopRawMutex, RefCell<I2c<'static, I2C0, i2c::Blocking>>>;
pub(crate) type I2c1Bus = AsyncMutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;
pub(crate) type Spi0Bus = Mutex<NoopRawMutex, RefCell<Spi<'static, SPI0, spi::Blocking>>>;

// Devices
pub(crate) type SharedBattery = AsyncMutex<NoopRawMutex, BatteryType>;
pub(crate) type Display = mipidsi::Display<
    SpiInterface<
        'static,
        SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, spi::Blocking>, Output<'static>>,
        Output<'static>,
    >,
    ILI9341Rgb565,
    Output<'static>,
>;

static SPI_0_BUS: StaticCell<Spi0Bus> = StaticCell::new();
static DISPLAY_BUFFER: StaticCell<[u8; 76800]> = StaticCell::new();

bind_interrupts!(struct I2C1Irqs {
    I2C1_IRQ => I2CInterruptHandler<I2C1>;
});

const DISPLAY_FREQ: u32 = 64_000_000;

#[derive(Debug)]
pub enum Error {
    // InitializationError,
    DisplayInitError,
    // RtcInitError,
    BatteryInitError,
}

pub fn init_rtc(
    bus: Peri<'static, I2C0>,
    scl: Peri<'static, PIN_21>,
    sda: Peri<'static, PIN_20>,
) -> RtcTimeSource<'static> {
    let i2c0_bus = I2c::new_blocking(bus, scl, sda, i2c::Config::default());
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let shared_i2c0_bus = I2C0_BUS.init(Mutex::new(RefCell::new(i2c0_bus)));
    let i2c_dev = BlockingI2cDevice::new(shared_i2c0_bus);
    let rtc_dev = Ds323x::new_ds3231(i2c_dev);
    // TODO Wrap in a Result
    RtcTimeSource::new(rtc_dev)
}

pub(crate) type BatteryType = AsyncIna219<
    AsyncI2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>,
    UnCalibrated,
>;

pub async fn init_battery(
    bus: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_7>,
    sda: Peri<'static, PIN_6>,
) -> Result<&'static SharedBattery, Error> {
    let i2c1_bus = I2c::new_async(bus, scl, sda, I2C1Irqs, i2c::Config::default());
    // This doesn't really need to be static (to be shared), as there is only one device on the but today.
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let shared_i2c1_bus = I2C1_BUS.init(AsyncMutex::new(i2c1_bus));
    let i2c_dev = AsyncI2cDevice::new(shared_i2c1_bus);
    let ina219 = AsyncIna219::new(i2c_dev, Address::from_byte(0x43).unwrap()).await;

    if let Ok(mut ina_dev) = ina219 {
        ina_dev.configuration().await.unwrap().conversion_time_us().unwrap();

        static INA_DEV: StaticCell<SharedBattery> = StaticCell::new();
        let ina_ref = INA_DEV.init(AsyncMutex::new(ina_dev));
        Ok(ina_ref)
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

pub fn init_shared_spi(
    bus: Peri<'static, SPI0>,
    miso: Peri<'static, PIN_16>,
    mosi: Peri<'static, PIN_19>,
    clk: Peri<'static, PIN_18>,
) -> &'static mut Spi0Bus {
    let spi_cfg = spi::Config::default();
    let spi = Spi::new_blocking(bus, clk, mosi, miso, spi_cfg);
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));
    SPI_0_BUS.init(spi_bus)
}

pub fn init_display(
    bus: &'static Spi0Bus,
    reset_pin: Peri<'static, PIN_27>,
    display_cs_pin: Peri<'static, PIN_17>,
    dc_pin: Peri<'static, PIN_26>,
) -> Result<Display, Error> {
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    let display_cs = Output::new(display_cs_pin, Level::High);
    let display_spi = SpiDeviceWithConfig::new(bus, display_cs, display_config);

    let dc = Output::new(dc_pin, Level::Low);
    let rst = Output::new(reset_pin, Level::Low);
    let buffer_ref = DISPLAY_BUFFER.init([0_u8; 76800]); // 240 * 320
    let display_interface = SpiInterface::new(display_spi, dc, buffer_ref);

    let display = Builder::new(ILI9341Rgb565, display_interface)
        .display_size(240, 320)
        .reset_pin(rst)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .init(&mut Delay);

    display.map_err(|_| Error::DisplayInitError)
}
