#![no_std]
#![no_main]

// Setup the allocator
extern crate alloc;
use embedded_alloc::LlffHeap as Heap;
// Create a global allocator instance
#[global_allocator]
static HEAP: Heap = Heap::empty();
// Reserve a static heap region (here: 32 KB)
const HEAP_SIZE: usize = 32 * 1024;
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

// Debug formatting
use defmt::*;
use defmt_rtt as _;

// Core functionalities
use alloc::{rc::Rc, string::String};
use core::{
    cell::{RefCell, RefMut, UnsafeCell},
    fmt::Write,
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering},
};
use critical_section::Mutex;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::MODE_0,
};
use embedded_sdmmc::{
    DirEntry, Error, Mode, SdCard, SdCardError, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
use ili9341::{Ili9341, Orientation};
use panic_probe as _;
use rp235x_hal::{
    // Clock,
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::{self, FunctionI2C, FunctionSpi, Pin},
    i2c::I2C,
    spi::Spi,
    {self as hal, entry},
};

// Peripherals
use display_interface_spi::SPIInterface;
use ds323x::{
    DateTimeAccess, Datelike, Ds323x, NaiveDateTime, Timelike, ic::DS3231, interface::I2cInterface,
};
use ina219::SyncIna219;
use ina219::address::Address;

// UI
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii},
    pixelcolor::{Rgb565, RgbColor},
    prelude::*,
    prelude::{Point, Primitive, Size},
    primitives::{Arc, Ellipse, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Text, renderer::CharacterStyle},
};

pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const MAX_BUS_VOLTAGE: u16 = 4200;
const MIN_BUS_VOLTAGE: u16 = 3800;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use some_bsp;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Pins setup
type Button1Pin = Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;
type Button2Pin = Pin<gpio::bank0::Gpio13, gpio::FunctionSioInput, gpio::PullUp>;
type EncoderAPin = Pin<gpio::bank0::Gpio2, gpio::FunctionSioInput, gpio::PullUp>;
type EncoderBPin = Pin<gpio::bank0::Gpio3, gpio::FunctionSioInput, gpio::PullUp>;

static BUTTON1_PIN: Mutex<RefCell<Option<Button1Pin>>> = Mutex::new(RefCell::new(None));
static BUTTON2_PIN: Mutex<RefCell<Option<Button2Pin>>> = Mutex::new(RefCell::new(None));
static ENCODER_A_PIN: Mutex<RefCell<Option<EncoderAPin>>> = Mutex::new(RefCell::new(None));
static ENCODER_B_PIN: Mutex<RefCell<Option<EncoderBPin>>> = Mutex::new(RefCell::new(None));

// Global States, buttons
static BUTTON1_STATE: AtomicBool = AtomicBool::new(false);
static BUTTON2_STATE: AtomicBool = AtomicBool::new(false);
// Encoder
static ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
static ENCODER_LAST_STATE: AtomicU8 = AtomicU8::new(0);

// Debounce
static BUTTON1_LAST: AtomicU32 = AtomicU32::new(0);
static BUTTON2_LAST: AtomicU32 = AtomicU32::new(0);
static ENCODER_LAST: AtomicU32 = AtomicU32::new(0);
static BUTTON_DEBOUNCE_DELAY_US: u32 = 200_000; // in microseconds
static ENCODER_DEBOUNCE_DELAY_US: u32 = 15_000; // in microseconds

struct RtcTimeSource<I2C> {
    // QUESTION: Do I have to pack it in a RefCell to be able to borrow it in the get_timestamp implementation?
    // Is there a simpler way to do it. To my understanding the ownership isn't the issue, but the TimeSource trait
    // constraint is the limitation.
    rtc: RefCell<Ds323x<I2cInterface<I2C>, DS3231>>,
}

impl<I2C> RtcTimeSource<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c_bus: I2C) -> Self {
        Self {
            rtc: RefCell::new(Ds323x::new_ds3231(i2c_bus)),
        }
    }

    fn with_rtc<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut RefMut<Ds323x<I2cInterface<I2C>, DS3231>>) -> R,
    {
        let mut rtc = self.rtc.borrow_mut();
        f(&mut rtc)
    }

    fn get_datetime(
        &self,
    ) -> Result<NaiveDateTime, ds323x::Error<<I2C as embedded_hal::i2c::ErrorType>::Error>> {
        self.with_rtc(|rtc| rtc.datetime())
    }
}

// This Newtype wrapper is required impl TimeSource on an Rc instance. See below when instantiating rtc_timesource.
pub struct SharedRtcTimeSource<I2C>(pub Rc<RtcTimeSource<I2C>>);

impl<I2C> Clone for SharedRtcTimeSource<I2C> {
    fn clone(&self) -> Self {
        SharedRtcTimeSource(self.0.clone())
    }
}

impl<I2C> TimeSource for SharedRtcTimeSource<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    fn get_timestamp(&self) -> Timestamp {
        if let Ok(dt) = self.0.with_rtc(|rtc| rtc.datetime()) {
            Timestamp {
                year_since_1970: (dt.year() as i32 - 1970) as u8,
                zero_indexed_month: (dt.month() as u8) - 1,
                zero_indexed_day: (dt.day() as u8) - 1,
                hours: dt.hour() as u8,
                minutes: dt.minute() as u8,
                seconds: dt.second() as u8,
            }
        } else {
            // Provide a sensible fallback in case of error.
            Timestamp {
                year_since_1970: 0,
                zero_indexed_month: 0,
                zero_indexed_day: 0,
                hours: 0,
                minutes: 0,
                seconds: 0,
            }
        }
    }
}

// Rust 2024 deny it by default. Allowance is required to initialize the allocator.
#[allow(static_mut_refs)]
#[entry]
fn main() -> ! {
    info!("Program start");

    // Initialize the allocator. Ensure this happen only once.
    unsafe {
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Buses
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

    let reset_pin = pins.gpio27.into_push_pull_output();
    let dc_pin = pins.gpio26.into_push_pull_output();
    let display_cs_pin = pins.gpio17.into_push_pull_output();
    let sd_cs_pin = pins.gpio8.into_push_pull_output();

    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        MODE_0,
    );
    let spi_bus = RefCell::new(spi);
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let exclusive_display_spi_dev =
        embedded_hal_bus::spi::RefCellDevice::new(&spi_bus, display_cs_pin, timer).unwrap();
    let exclusive_sd_spi_dev =
        embedded_hal_bus::spi::RefCellDevice::new(&spi_bus, sd_cs_pin, timer).unwrap();

    let display_iface = SPIInterface::new(exclusive_display_spi_dev, dc_pin);

    //
    // Interrupts
    //

    // let button1_pin = pins.gpio12.reconfigure();
    let button1_pin = pins.gpio12.into_pull_up_input();
    let button2_pin = pins.gpio13.into_pull_up_input();

    let encoder_a_pin = pins.gpio2.into_pull_up_input();
    let encoder_b_pin = pins.gpio3.into_pull_up_input();
    // let mut encoder_led_pin = pins.gpio4.into_push_pull_output();
    // let mut encoder_switch_pin = pins.gpio5.into_pull_up_input();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    button1_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    button2_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    // For encoders, both edges are relevant
    encoder_a_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    encoder_a_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    encoder_b_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    encoder_b_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    // Export to global mutexes
    critical_section::with(|cs| {
        BUTTON1_PIN.borrow(cs).replace(Some(button1_pin));
        BUTTON2_PIN.borrow(cs).replace(Some(button2_pin));
        ENCODER_A_PIN.borrow(cs).replace(Some(encoder_a_pin));
        ENCODER_B_PIN.borrow(cs).replace(Some(encoder_b_pin));
    });

    // Initialize encoder state
    ENCODER_LAST_STATE.store(0, Ordering::Relaxed);

    // Unmask the IRQ for I/O Bank 0 so that the RP2350's interrupt controller
    // (NVIC in Arm mode, or Xh3irq in RISC-V mode) will jump to the interrupt
    // function when the interrupt occurs. We do this last so that the interrupt
    // can't go off while it is in the middle of being configured
    unsafe {
        // FIXME: This isn't the code provided in examples. Example provided in HEAD are not available in 0.3.0, update
        // it with the next rp235x_hal crate update.
        // hal::arch::interrupt_unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    }

    // Enable interrupts on this core
    unsafe {
        hal::arch::interrupt_enable();
    }

    // RTC
    let rtc_timesource = SharedRtcTimeSource(Rc::new(RtcTimeSource::new(rtc_i2c)));
    // Set time manually:
    // let begin = NaiveDate::from_ymd_opt(2025, 8, 21)
    //     .unwrap()
    //     .and_hms_opt(15, 20, 10)
    //     .unwrap();
    // rtc.set_datetime(&begin).unwrap();

    // Battery
    let mut ina = None;
    let mut ina219 = SyncIna219::new(battery_i2c, Address::from_byte(0x43).unwrap());
    if let Ok(ref mut ina_dev) = ina219 {
        ina_dev
            .configuration()
            .unwrap()
            .conversion_time_us()
            .unwrap();
        ina = Some(ina_dev)
    } else {
        info!("Failed to setup the battery chip management.")
    }

    // Display
    let mut backlight_pin = pins.gpio15.into_push_pull_output();

    let mut display = Ili9341::new(
        display_iface,
        reset_pin,
        &mut timer,
        Orientation::LandscapeFlipped,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    let bg_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    let battery_bar_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::CSS_AQUA)
        .build();

    let debug_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::CSS_PINK)
        .stroke_width(1)
        .fill_color(Rgb565::CSS_GRAY)
        .build();

    Rectangle::new(Point::new(0, 0), Size::new(320, 240))
        .into_styled(bg_style)
        .draw(&mut display)
        .unwrap();

    let mut text_style = MonoTextStyle::new(&ascii::FONT_10X20, Rgb565::BLUE);

    Text::new(
        "Verdigris, Music Player, Berlin 07.2025",
        Point::new(5, 20),
        text_style,
    )
    .draw(&mut display)
    .unwrap();

    // text_style.set_text_color(Some(Rgb565::WHITE));
    // text_style.set_text_color(Some(Rgb565::RED));

    // Turn on the display
    backlight_pin.set_high().unwrap();

    // SD-Card
    let sdcard = SdCard::new(exclusive_sd_spi_dev, timer);
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    let volume_mgr = VolumeManager::new(sdcard, rtc_timesource.clone());
    let shared_rtc = rtc_timesource.0;

    let raw_volume0 = volume_mgr.open_raw_volume(VolumeIdx(0)).unwrap();

    let root_dir = volume_mgr.open_root_dir(raw_volume0).unwrap();
    let print = |item: &DirEntry| {
        let filename = core::str::from_utf8(item.name.base_name()).unwrap();
        info!("{:?} {:?} Mb", filename, item.size as f32 / 1024.0 / 1024.0)
    };
    volume_mgr.iterate_dir(root_dir, print).unwrap();

    let mut previous_datetime = shared_rtc.get_datetime().unwrap().and_utc();

    let row_height = 20; // pixels
    let datetime_row = 2 * row_height;
    let battery_row = 3 * row_height;
    let battery_bar_row = 4 * row_height;
    let buttons_row = 5 * row_height;
    let button_dot_size = 25;

    loop {
        // DEBUG: wait for interrupt (to test)
        // hal::arch::wfi();

        // Get and display the time
        let dt = shared_rtc.get_datetime().unwrap();
        if previous_datetime.timestamp() % 100 != dt.and_utc().timestamp() % 100 {
            let mut line: String = String::new();
            core::write!(
                &mut line,
                "{}/{:02}/{:02} {:02}:{:02}:{:02}",
                dt.year(),
                dt.month(),
                dt.day(),
                dt.hour(),
                dt.minute(),
                dt.second(),
            )
            .unwrap();

            Rectangle::new(
                Point::new(0, datetime_row - row_height + 2),
                Size::new(320, row_height as u32),
            )
            .into_styled(bg_style)
            .draw(&mut display)
            .unwrap();

            text_style.set_text_color(Some(Rgb565::WHITE));
            Text::new(line.as_str(), Point::new(5, datetime_row), text_style)
                .draw(&mut display)
                .unwrap();

            previous_datetime = dt.and_utc();
        }

        // Get and display the battery status
        if let Some(ref mut ina_dev) = ina {
            Rectangle::new(
                Point::new(0, battery_row - row_height + 2),
                Size::new(320, row_height as u32),
            )
            .into_styled(bg_style)
            .draw(&mut display)
            .unwrap();

            let shunt = ina_dev.shunt_voltage().unwrap();
            let bus = ina_dev.bus_voltage().unwrap();
            let bar_resolution = (MAX_BUS_VOLTAGE - MIN_BUS_VOLTAGE) as f32;
            let bar_percentage =
                100.0 / bar_resolution * (bus.voltage_mv() - MIN_BUS_VOLTAGE) as f32;
            // This averages a bit the percentage and prevents flickering
            let bar_width = bar_percentage * 3.2;

            if shunt.shunt_voltage_mv() < 0 {
                text_style.set_text_color(Some(Rgb565::RED));
            } else {
                text_style.set_text_color(Some(Rgb565::GREEN));
            }

            let mut percentage_line: String = String::new();
            core::write!(&mut percentage_line, "{}%", bar_percentage).unwrap();
            // info!("{}", bus_line.as_str());
            Text::new(
                percentage_line.as_str(),
                Point::new(5, battery_row),
                text_style,
            )
            .draw(&mut display)
            .unwrap();

            let mut bus_line: String = String::new();
            core::write!(&mut bus_line, "Bus: {} mV", bus.voltage_mv()).unwrap();
            // info!("{}", bus_line.as_str());
            Text::new(bus_line.as_str(), Point::new(50, battery_row), text_style)
                .draw(&mut display)
                .unwrap();

            let mut shunt_line: String = String::new();
            core::write!(&mut shunt_line, "Shunt: {} mV", shunt.shunt_voltage_mv()).unwrap();
            // info!("{}", shunt_line.as_str());
            Text::new(
                shunt_line.as_str(),
                Point::new(185, battery_row),
                text_style,
            )
            .draw(&mut display)
            .unwrap();

            Rectangle::new(
                Point::new(0, battery_bar_row - (row_height / 2) + 2),
                Size::new(320 as u32, 2),
            )
            .into_styled(bg_style)
            .draw(&mut display)
            .unwrap();
            Rectangle::new(
                Point::new(0, battery_bar_row - (row_height / 2) + 2),
                Size::new(bar_width as u32, 2),
            )
            .into_styled(battery_bar_style)
            .draw(&mut display)
            .unwrap();
        } else {
            text_style.set_text_color(Some(Rgb565::RED));
            Text::new("No battery", Point::new(5, battery_row), text_style)
                .draw(&mut display)
                .unwrap();
        }

        // TODO: rewrite DRY
        let button1_style = if BUTTON1_STATE.load(Ordering::Relaxed) {
            PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build()
        } else {
            PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::WHITE)
                .build()
        };

        Ellipse::new(
            Point::new(5, buttons_row),
            Size::new(button_dot_size, button_dot_size),
        )
        .into_styled(button1_style)
        .draw(&mut display)
        .unwrap();

        let button2_style = if BUTTON2_STATE.load(Ordering::Relaxed) {
            PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build()
        } else {
            PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::WHITE)
                .build()
        };

        Ellipse::new(
            Point::new(50, buttons_row),
            Size::new(button_dot_size, button_dot_size),
        )
        .into_styled(button2_style)
        .draw(&mut display)
        .unwrap();

        let encoder_value = ENCODER_COUNT.load(Ordering::Relaxed) as f32;

        Arc::new(
            Point::new(100, buttons_row),
            20,
            -45_f32.deg(),
            (-45.0 + encoder_value).deg(),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::YELLOW, 2))
        .draw(&mut display)
        .unwrap();

        timer.delay_ms(100);
    }
}

#[inline(always)]
fn now_us() -> u32 {
    // Safe because we're only *reading* from a monotonic counter
    let timer = unsafe { &*hal::pac::TIMER0::ptr() };
    timer.timerawl().read().bits() // lower 32 bits of the free-running timer
}

/// This is the interrupt handler that fires when GPIO Bank 0 detects an event
/// (like an edge).
///
/// We give it an unmangled name so that it replaces the default (empty)
/// handler. These handlers are referred to by name from the Interrupt Vector
/// Table created by cortex-m-rt.
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
fn IO_IRQ_BANK0() {
    // Enter a critical section to ensure this code cannot be concurrently
    // executed on the other core. This also protects us if the main thread
    // decides to execute this function (which it shouldn't, but we can't stop
    // them if they wanted to).

    critical_section::with(|cs| {
        let now_us = now_us();

        // FIXME: not DRY
        // BUTTON1
        if let Some(ref mut btn1) = BUTTON1_PIN.borrow(cs).borrow_mut().as_mut() {
            if btn1.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
                let last = BUTTON1_LAST.load(Ordering::Relaxed);
                if now_us.wrapping_sub(last) > BUTTON_DEBOUNCE_DELAY_US {
                    let cur = BUTTON1_STATE.load(Ordering::Relaxed);
                    BUTTON1_STATE.store(!cur, Ordering::Relaxed);
                    BUTTON1_LAST.store(now_us, Ordering::Relaxed);
                }
                btn1.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            }
        }

        // BUTTON2
        if let Some(ref mut btn2) = BUTTON2_PIN.borrow(cs).borrow_mut().as_mut() {
            if btn2.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
                let last = BUTTON2_LAST.load(Ordering::Relaxed);
                if now_us.wrapping_sub(last) > BUTTON_DEBOUNCE_DELAY_US {
                    let cur = BUTTON2_STATE.load(Ordering::Relaxed);
                    BUTTON2_STATE.store(!cur, Ordering::Relaxed);
                    BUTTON2_LAST.store(now_us, Ordering::Relaxed);
                }
                btn2.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            }
        }

        // ENCODER
        // FIXME: Not Dry
        let mut new_state = 0u8;

        // FIXME: There is much better ways to debounce the encoder, either in software or with a schmitt-trigger.
        // Using an IC will increase the BoM, but should be more reliable, and spares interrupts and CPU cycles.
        let last = ENCODER_LAST.load(Ordering::Relaxed);
        if now_us.wrapping_sub(last) > ENCODER_DEBOUNCE_DELAY_US {
            if let Some(ref mut enc_a) = ENCODER_A_PIN.borrow(cs).borrow_mut().as_mut() {
                if enc_a.interrupt_status(hal::gpio::Interrupt::EdgeLow)
                    || enc_a.interrupt_status(hal::gpio::Interrupt::EdgeHigh)
                {
                    enc_a.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
                    enc_a.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
                }
                if enc_a.is_high().unwrap_or(false) {
                    new_state |= 0b10;
                }
            }

            if let Some(ref mut enc_b) = ENCODER_B_PIN.borrow(cs).borrow_mut().as_mut() {
                if enc_b.interrupt_status(hal::gpio::Interrupt::EdgeLow)
                    || enc_b.interrupt_status(hal::gpio::Interrupt::EdgeHigh)
                {
                    enc_b.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
                    enc_b.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
                }
                if enc_b.is_high().unwrap_or(false) {
                    new_state |= 0b01;
                }
            }

            let last = ENCODER_LAST_STATE.load(Ordering::Relaxed);

            // Quadrature decoding table
            // (last_state << 2 | new_state) → delta
            const LOOKUP: [i32; 16] = [
                0, -1, 1, 0, // 00
                1, 0, 0, -1, // 01
                -1, 0, 0, 1, // 10
                0, 1, -1, 0, // 11
            ];

            let idx = (last << 2 | new_state) as usize;
            let delta = LOOKUP[idx];
            if delta != 0 {
                ENCODER_COUNT.fetch_add(delta, Ordering::Relaxed);
                ENCODER_LAST_STATE.store(new_state, Ordering::Relaxed);
            }
            ENCODER_LAST.store(now_us, Ordering::Relaxed);
        }
    });
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"Verdigris Music Player, Prototype Zero"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];
