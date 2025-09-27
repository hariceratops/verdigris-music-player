#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::info;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embassy_embedded_hal::shared_bus::asynch::spi::{SpiDevice, SpiDeviceWithConfig};
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c, InterruptHandler as I2CInterruptHandler},
    peripherals::{I2C0, I2C1, PIO0, SPI0},
    pio::{InterruptHandler as HIDInterruptHandler, Pio},
    pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
    spi::{self, Spi},
};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::NoopRawMutex},
    mutex::Mutex as AsyncMutex,
};
use embassy_time::{Delay, Duration, Instant, Timer, with_deadline};
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use static_cell::StaticCell;

use mipidsi::Builder;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{Orientation, Rotation};

use {defmt_rtt as _, panic_probe as _};

// Modules
mod bus;
mod debouncer;
mod hid;
mod rtc;

use crate::bus::BatteryType;
use crate::debouncer::Debouncer;
use crate::rtc::RtcTimeSource;

const DISPLAY_FREQ: u32 = 64_000_000;

// Busses
pub(crate) type I2c0Bus = Mutex<NoopRawMutex, RefCell<I2c<'static, I2C0, i2c::Blocking>>>;
pub(crate) type I2c1Bus = AsyncMutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;

// Devices
pub(crate) type SharedBattery = AsyncMutex<NoopRawMutex, BatteryType>;

bind_interrupts!(struct HIDIrqs {
    PIO0_IRQ_0 => HIDInterruptHandler<PIO0>;
});

bind_interrupts!(struct I2C0Irqs {
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
});

bind_interrupts!(struct I2C1Irqs {
    I2C1_IRQ => I2CInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // HID
    let button1 = Debouncer::new(Input::new(p.PIN_12, Pull::Up), Duration::from_millis(20));
    let button2 = Debouncer::new(Input::new(p.PIN_13, Pull::Up), Duration::from_millis(20));

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, HIDIrqs);

    let pio_encoder_program = PioEncoderProgram::new(&mut common);
    let pio_encoder = PioEncoder::new(&mut common, sm0, p.PIN_2, p.PIN_3, &pio_encoder_program);

    spawner.spawn(hid::encoder_job(pio_encoder)).unwrap();
    spawner.spawn(hid::button1_job(button1)).unwrap();
    spawner.spawn(hid::button2_job(button2)).unwrap();

    // Busses
    // DOC: https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/shared_bus.rs

    // Shared I2C bus
    let i2c0_bus = I2c::new_blocking(p.I2C0, p.PIN_21, p.PIN_20, i2c::Config::default());
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let shared_i2c0_bus = I2C0_BUS.init(Mutex::new(RefCell::new(i2c0_bus)));
    let mut rtc_dev = bus::init_rtc(shared_i2c0_bus);
    let rtc_timesource = RtcTimeSource::new(rtc_dev);

    spawner.spawn(bus::rtc_job(shared_i2c0_bus)).unwrap();

    let i2c1_bus = I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, I2C1Irqs, i2c::Config::default());
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let shared_i2c1_bus = I2C1_BUS.init(AsyncMutex::new(i2c1_bus));
    static INA_DEV: StaticCell<SharedBattery> = StaticCell::new();
    let shared_battery = bus::init_battery(shared_i2c1_bus).await.unwrap();
    let ina_ref = INA_DEV.init(AsyncMutex::new(shared_battery));

    spawner.spawn(bus::battery_job(ina_ref)).unwrap();

    // let mut display = bus::init_display(spi_bus, display_cs_pin).unwrap();
    // spawner.spawn(bus::init_display(spi_bus, cs).unwrap());

    // Shared SPI bus
    let spi_cfg = spi::Config::default();

    let backlight_pin = p.PIN_15;
    let reset_pin = p.PIN_27;
    let display_cs_pin = p.PIN_17;
    let sd_cs_pin = p.PIN_8;
    let dc_pin = p.PIN_26;
    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;

    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    let spi = Spi::new_blocking(p.SPI0, clk, mosi, miso, spi_cfg);
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let display_cs = Output::new(display_cs_pin, Level::High);
    let sd_cs = Output::new(sd_cs_pin, Level::High);

    let display_spi = SpiDeviceWithConfig::new(&spi_bus, display_cs, display_config);

    let dc = Output::new(dc_pin, Level::Low);
    let rst = Output::new(reset_pin, Level::Low);
    let mut backlight = Output::new(backlight_pin, Level::Low);

    let mut buffer = [0_u8; 76800]; // 240 * 320
    let display_interface = SpiInterface::new(display_spi, dc, &mut buffer);

    // let mut display = Builder::new(ST7789, display_interface)
    let mut display = Builder::new(ILI9341Rgb565, display_interface)
        .display_size(240, 320)
        .reset_pin(rst)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .init(&mut Delay)
        .unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    backlight.set_high();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(34, 68));

    // Display the image
    ferris.draw(&mut display).unwrap();

    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    Text::new(
        "Hello embedded_graphics \n + embassy + RP2040!",
        Point::new(20, 200),
        style,
    )
    .draw(&mut display)
    .unwrap();

    let mut previous_datetime = rtc_timesource.get_datetime().unwrap().and_utc();

    loop {
        info!("Previous datetime {:?}", previous_datetime.timestamp());
        Timer::after_secs(1).await;
        previous_datetime = rtc_timesource.get_datetime().unwrap().and_utc();
    }
}
