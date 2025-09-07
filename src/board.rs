use alloc::rc::Rc;
use display_interface_spi::SPIInterface;
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use ili9341::{Ili9341, Orientation};
use ina219::{
    SyncIna219,
    address::Address,
    calibration::{UnCalibrated},
};
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

use crate::rtc::{RtcTimeSource, SharedRtcTimeSource};

// External high-speed crystal on the pico board is 12Mhz
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[derive(Debug)]
pub enum Error {
    // InitializationError,
    DisplayInitError,
    // RtcInitError,
    BatteryInitError,
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

type RtcI2C = hal::I2C<I2C0, RtcI2cPins>;
type BatteryI2C = hal::I2C<I2C1, BatteryI2cPins>;
pub type SpiBusTypeAlias = Spi<Enabled, SPI0, SpiPins>;
type DisplayTypeAlias<'a> = Ili9341<
    SPIInterface<
        AtomicDevice<'a, SpiBusTypeAlias, DisplayCsPin, Timer<CopyableTimer0>>,
        SpiDcPin,
    >,
    SpiResetPin,
>;

pub(crate) struct Display<'a> {
    pub(crate) ili: DisplayTypeAlias<'a>,
    pub(crate) backlight_pin: DisplayBacklightPin,
}

pub struct Board {
    pub(crate) display_backlight_pin: Option<DisplayBacklightPin>,
    pub(crate) button1_pin: Option<Button1Pin>,
    pub(crate) button2_pin: Option<Button2Pin>,
    pub(crate) encoder_a_pin: Option<EncoderAPin>,
    pub(crate) encoder_b_pin: Option<EncoderBPin>,
    pub(crate) dc_pin: Option<SpiDcPin>,
    pub(crate) reset_pin: Option<SpiResetPin>,
    pub(crate) display_cs_pin: Option<DisplayCsPin>,
    pub(crate) timer: Option<Timer<CopyableTimer0>>,
    pub(crate) rtc_i2c: Option<RtcI2C>,
    pub(crate) battery_i2c: Option<BatteryI2C>,
    pub(crate) sd_cs_pin: Option<SdCsPin>,
    pub(crate) spi_bus: Option<AtomicCell<SpiBusTypeAlias>>,
    // display: DisplayTypeAlias
}

impl Board {
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
        rtc_i2c: RtcI2C,
        battery_i2c: BatteryI2C,
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

    pub fn init() -> Result<Board, Error> {
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

        // TODO: Return an error, eventually
        Ok(ports)
    }

    pub fn take_display_backlight_pin(&mut self) -> DisplayBacklightPin {
        self.display_backlight_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_button1_pin(&mut self) -> Button1Pin {
        self.button1_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_button2_pin(&mut self) -> Button2Pin {
        self.button2_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_encoder_a_pin(&mut self) -> EncoderAPin {
        self.encoder_a_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_encoder_b_pin(&mut self) -> EncoderBPin {
        self.encoder_b_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_dc_pin(&mut self) -> SpiDcPin {
        self.dc_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_reset_pin(&mut self) -> SpiResetPin {
        self.reset_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_display_cs_pin(&mut self) -> DisplayCsPin {
        self.display_cs_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_timer(&mut self) -> Timer<CopyableTimer0> {
        self.timer
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_rtc_i2c(&mut self) -> RtcI2C {
        self.rtc_i2c
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_battery_i2c(&mut self) -> BatteryI2C {
        self.battery_i2c
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_sd_cs_pin(&mut self) -> SdCsPin {
        self.sd_cs_pin
            .take()
            .expect("Battery I2C can only be taken once!")
    }
    pub fn take_spi_bus(&mut self) -> AtomicCell<SpiBusTypeAlias> {
        self.spi_bus
            .take()
            .expect("Battery I2C can only be taken once!")
    }

    pub fn init_display<'a>(
        &mut self,
        spi_bus: &'a AtomicCell<SpiBusTypeAlias>,
        timer: &'a mut Timer<CopyableTimer0>,
    ) -> Result<Display<'a>, Error> {
        let dc_pin = self.take_dc_pin();
        let reset_pin = self.take_reset_pin();
        let display_cs_pin = self.take_display_cs_pin();
        let display_backlight_pin = self.take_display_backlight_pin();

        let exclusive_display_spi_dev =
            AtomicDevice::new(spi_bus, display_cs_pin, timer.clone()).unwrap();

        let display_iface = SPIInterface::new(exclusive_display_spi_dev, dc_pin);
        if let Ok(ili) = Ili9341::new(
            display_iface,
            reset_pin,
            timer,
            Orientation::LandscapeFlipped,
            ili9341::DisplaySize240x320,
        ) {
            // STORY: Borrowed value timer didn't outlive display
            Ok(Display {
                ili,
                backlight_pin: display_backlight_pin
            })
        } else {
            Err(Error::DisplayInitError)
        }
    }

    pub fn init_battery(&mut self) -> Result<SyncIna219<BatteryI2C, UnCalibrated>, Error> {
        let battery_i2c = self.take_battery_i2c();
        let ina219 = SyncIna219::new(battery_i2c, Address::from_byte(0x43).unwrap());
        if let Ok(mut ina_dev) = ina219 {
            ina_dev
                .configuration()
                .unwrap()
                .conversion_time_us()
                .unwrap();
            Ok(ina_dev)
        } else {
            Err(Error::BatteryInitError)
        }
    }
    pub fn init_rtc_timesource(&mut self) -> Option<SharedRtcTimeSource<RtcI2C>> {
        let rtc_i2c = self.take_rtc_i2c();
        let shared_rtc = SharedRtcTimeSource(Rc::new(RtcTimeSource::new(rtc_i2c)));
        Some(shared_rtc)
    }
}
