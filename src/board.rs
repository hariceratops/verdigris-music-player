use core::cell::RefCell;
use cortex_m::interrupt::CriticalSection;
use critical_section::Mutex;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii},
    pixelcolor::{Rgb565, RgbColor},
    prelude::*,
    prelude::{Point, Primitive, Size},
    primitives::{Arc, Ellipse, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Text, renderer::CharacterStyle},
};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use ili9341::{Ili9341, Orientation};
use ina219::{calibration::{Calibration, UnCalibrated}, SyncIna219};
use panic_probe as _;
use rp235x_hal::{
    // Clock,
    self as hal,
    Sio,
    Timer,
    Watchdog,
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::{
        FunctionI2C, FunctionSioInput, FunctionSioOutput, FunctionSpi, Pin, PullDown, PullUp,
        bank0::*,
    },
    i2c::I2C,
    pac::{I2C0, I2C1, Peripherals, SPI0},
    spi::{Enabled, Spi},
    timer::CopyableTimer0,
};

// External high-speed crystal on the pico board is 12Mhz
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[derive(Debug)]
pub enum Error {
    InitializationError,
    DisplayInitError,
    RtcInitError,
    BatteryInitError
}

pub type Button1Pin = Pin<Gpio12, FunctionSioInput, PullUp>;
pub type Button2Pin = Pin<Gpio13, FunctionSioInput, PullUp>;
pub type EncoderAPin = Pin<Gpio2, FunctionSioInput, PullUp>;
pub type EncoderBPin = Pin<Gpio3, FunctionSioInput, PullUp>;

pub type SpiDcPin = Pin<Gpio26, FunctionSioOutput, PullDown>;
pub type SpiResetPin = Pin<Gpio27, FunctionSioOutput, PullDown>;

type RtcI2cPins = (
    Pin<Gpio20, FunctionI2C, PullUp>,
    Pin<Gpio21, FunctionI2C, PullUp>,
);
type BatteryI2cPins = (
    Pin<Gpio6, FunctionI2C, PullUp>,
    Pin<Gpio7, FunctionI2C, PullUp>,
);
type SpiPins = (
    Pin<Gpio19, FunctionSpi, PullDown>,
    Pin<Gpio16, FunctionSpi, PullDown>,
    Pin<Gpio18, FunctionSpi, PullDown>,
);

type DisplayCsPin = Pin<Gpio17, FunctionSioOutput, PullDown>;
pub type DisplayBacklightPin = Pin<Gpio15, FunctionSioOutput, PullDown>;
type SdCsPin = Pin<Gpio8, FunctionSioOutput, PullDown>;

type SpiBusTypeAlias = Spi<Enabled, SPI0, SpiPins>;
type DisplayTypeAlias = Ili9341<
    SPIInterface<
        AtomicDevice<'static, SpiBusTypeAlias, DisplayCsPin, Timer<CopyableTimer0>>,
        SpiDcPin,
    >,
    SpiResetPin,
>;

pub static PORTS: Mutex<RefCell<Option<Ports>>> = Mutex::new(RefCell::new(None));

pub struct Ports {
    pub(crate) display_backlight_pin: Option<DisplayBacklightPin>,
    pub(crate) button1_pin: Option<Button1Pin>,
    pub(crate) button2_pin: Option<Button2Pin>,
    pub(crate) encoder_a_pin: Option<EncoderAPin>,
    pub(crate) encoder_b_pin: Option<EncoderBPin>,
    pub(crate) dc_pin: Option<SpiDcPin>,
    pub(crate) reset_pin: Option<SpiResetPin>,
    pub(crate) display_cs_pin: Option<DisplayCsPin>,
    pub(crate) timer: Option<Timer<CopyableTimer0>>,
    pub(crate) rtc_i2c: Option<hal::I2C<I2C0, RtcI2cPins>>,
    pub(crate) battery_i2c: Option<hal::I2C<I2C1, BatteryI2cPins>>,
    pub(crate) sd_cs_pin: Option<SdCsPin>,
    pub(crate) spi_bus: Option<AtomicCell<SpiBusTypeAlias>>,
    // display: DisplayTypeAlias
}

impl Ports {
    fn new(
        display_backlight_pin: DisplayBacklightPin,
        button1_pin: Button1Pin,
        button2_pin: Button2Pin,
        encoder_a_pin: EncoderAPin,
        encoder_b_pin: EncoderBPin,
        dc_pin: SpiDcPin,
        reset_pin: SpiResetPin,
        display_cs_pin: DisplayCsPin,
        timer: Timer<CopyableTimer0>,
        rtc_i2c: hal::I2C<I2C0, RtcI2cPins>,
        battery_i2c: hal::I2C<I2C1, BatteryI2cPins>,
        sd_cs_pin: SdCsPin,
        spi_bus: AtomicCell<SpiBusTypeAlias>,
    ) -> Self {
        Self {
            display_backlight_pin: Some(display_backlight_pin),
            button1_pin: Some(button1_pin),
            button2_pin: Some(button2_pin),
            encoder_a_pin: Some(encoder_a_pin),
            encoder_b_pin: Some(encoder_b_pin),
            dc_pin: Some(dc_pin),
            reset_pin: Some(reset_pin),
            display_cs_pin: Some(display_cs_pin),
            timer: Some(timer),
            rtc_i2c: Some(rtc_i2c),
            battery_i2c: Some(battery_i2c),
            sd_cs_pin: Some(sd_cs_pin),
            spi_bus: Some(spi_bus),
        }
    }

    pub fn init() -> Result<(), Error> {
        // Share pac, sio and pins in a struct crate-wide

        // Initialize HAL GPIOs and Timers
        let mut pac = Peripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);

        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Timers
        let timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

        //
        // GPIOs
        //
        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let display_backlight_pin = pins.gpio15.into_push_pull_output();

        //
        // Buttons
        //
        let button1_pin = pins.gpio12.into_pull_up_input();
        let button2_pin = pins.gpio13.into_pull_up_input();

        let encoder_a_pin = pins.gpio2.into_pull_up_input();
        let encoder_b_pin = pins.gpio3.into_pull_up_input();
        // let mut encoder_led_pin = pins.gpio4.into_push_pull_output();
        // let mut encoder_switch_pin = pins.gpio5.into_pull_up_input();

        //
        // Buses
        //
        // I2C
        let rtc_sda_pin: Pin<_, FunctionI2C, _> = pins.gpio20.reconfigure();
        let rtc_scl_pin: Pin<_, FunctionI2C, _> = pins.gpio21.reconfigure();
        let rtc_i2c = I2C::i2c0(
            pac.I2C0,
            rtc_sda_pin,
            rtc_scl_pin,
            400.kHz(),
            &mut pac.RESETS,
            &clocks.system_clock,
        );

        let battery_sda_pin: Pin<_, FunctionI2C, _> = pins.gpio6.reconfigure();
        let battery_scl_pin: Pin<_, FunctionI2C, _> = pins.gpio7.reconfigure();
        let battery_i2c = I2C::i2c1(
            pac.I2C1,
            battery_sda_pin,
            battery_scl_pin,
            400.kHz(),
            &mut pac.RESETS,
            &clocks.system_clock,
        );

        // SPI
        let mosi = pins.gpio19.into_function::<FunctionSpi>();
        let miso = pins.gpio16.into_function::<FunctionSpi>();
        let sclk = pins.gpio18.into_function::<FunctionSpi>();
        let spi_pin_layout = (mosi, miso, sclk);

        let dc_pin = pins.gpio26.into_push_pull_output();
        let reset_pin = pins.gpio27.into_push_pull_output();
        let display_cs_pin = pins.gpio17.into_push_pull_output();
        let sd_cs_pin = pins.gpio8.into_push_pull_output();

        let spi = Spi::<_, _, _, 8>::new(pac.SPI0, spi_pin_layout).init(
            &mut pac.RESETS,
            125_000_000u32.Hz(),
            16_000_000u32.Hz(),
            MODE_0,
        );

        // DOC: https://deepwiki.com/rust-embedded/embedded-hal/4.1-spi-bus-sharing
        let spi_bus = AtomicCell::new(spi);

        // NOTE: I wasn't able to instantiate SPI exclusive devices here because they rely on borrowed value that won't
        // outlive the exported instances.

        let ports = Self::new(
            display_backlight_pin,
            button1_pin,
            button2_pin,
            encoder_a_pin,
            encoder_b_pin,
            dc_pin,
            reset_pin,
            display_cs_pin,
            timer,
            rtc_i2c,
            battery_i2c,
            sd_cs_pin,
            spi_bus,
        );

        // Export busses
        critical_section::with(move |cs| {
            PORTS.borrow(cs).borrow_mut().replace(ports);
        });

        Ok(())
    }

    // pub fn init_display(
    //     spi_bus_ref: &AtomicCell<SpiBusTypeAlias>,
    // ) -> Result<(DisplayTypeAlias, DisplayBacklightPin), Error> {
    //     let (dc_pin, reset_pin, display_cs_pin, display_backlight_pin, mut timer) =
    //         get_ports(|ports| {
    //             (
    //                 ports.dc_pin.take().unwrap(),
    //                 ports.reset_pin.take().unwrap(),
    //                 ports.display_cs_pin.take().unwrap(),
    //                 ports.display_backlight_pin.take().unwrap(),
    //                 ports.timer.take().unwrap(),
    //             )
    //         })
    //         .unwrap();
    //     let exclusive_display_spi_dev =
    //         AtomicDevice::new(spi_bus_ref, display_cs_pin, timer).unwrap();
    //
    //     let display_iface = SPIInterface::new(exclusive_display_spi_dev, dc_pin);
    //     if let Ok(ili) = Ili9341::new(
    //         display_iface,
    //         reset_pin,
    //         &mut timer,
    //         Orientation::LandscapeFlipped,
    //         ili9341::DisplaySize240x320,
    //     ) {
    //         // TODO: Borrowed value timer doesn't outlive display
    //         Ok((ili, display_backlight_pin))
    //     } else {
    //         Err(Error::DisplayInitError)
    //     }
    // }

    // pub fn init_battery() -> Result<SyncIna219<hal::I2C<I2C0, RtcI2cPins>, UnCalibrated>, Error> {
    //     let mut ina219 = SyncIna219::new(battery_i2c, Address::from_byte(0x43).unwrap());
    //     if let Ok(ref mut ina_dev) = ina219 {
    //         ina_dev
    //             .configuration()
    //             .unwrap()
    //             .conversion_time_us()
    //             .unwrap();
    //         Ok(ina_dev)
    //     } else {
    //         Err(Error::BatteryInitError)
    //     }
    // }
}

pub fn get_ports<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut Ports) -> R,
{
    critical_section::with(|cs| {
        if let Some(ports) = PORTS.borrow(cs).borrow_mut().as_mut() {
            Some(f(ports))
        } else {
            None
        }
    })
}

// let (
//     spi_bus,
//     dc_pin,
//     reset_pin,
//     display_cs_pin,
//     mut display_backlight_pin,
//     sd_cs_pin,
//     rtc_timesource,
//     battery_i2c,
//     mut timer,
// ) = board::get_ports(|ports| {
//     (
//         // Display
//         ports.spi_bus.take().unwrap(),
//         ports.dc_pin.take().unwrap(),
//         ports.reset_pin.take().unwrap(),
//         ports.display_cs_pin.take().unwrap(),
//         ports.display_backlight_pin.take().unwrap(),
//         // SD-Card
//         ports.sd_cs_pin.take().unwrap(),
//         // RTC device
//         SharedRtcTimeSource(Rc::new(RtcTimeSource::new(ports.rtc_i2c.take().unwrap()))),
//         // Battery device
//         ports.battery_i2c.take().unwrap(),
//         // Common
//         ports.timer.take().unwrap(),
//     )
// })
// .unwrap();
