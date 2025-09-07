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
    cell::RefCell,
    fmt::Write,
    sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering},
};
use critical_section::Mutex;
use embedded_hal_bus::spi::AtomicDevice;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::{SpiBus, MODE_0},
};
use embedded_sdmmc::{DirEntry, SdCard, VolumeIdx, VolumeManager};
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

mod rtc;
use rtc::{RtcTimeSource, SharedRtcTimeSource};

mod interrupts;
use interrupts::{BUTTON1_STATE, BUTTON2_STATE, ENCODER_COUNT};

mod board;
use board::Ports;

// Battery
const MAX_BUS_VOLTAGE: u16 = 4200;
const MIN_BUS_VOLTAGE: u16 = 3800;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use some_bsp;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Rust 2024 deny it by default. Allowance is required to initialize the allocator.
#[allow(static_mut_refs)]
#[entry]
fn main() -> ! {
    info!("Program start");

    // Initialize the allocator. Ensure this happen only once.
    unsafe {
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    // Board::init();
    // BoardAccessor::get()
    // BoardAccessor::get_display()
    Ports::init().expect("Failed to setup boards");

    if let Ok(()) = interrupts::init() {
        info!("Interrupts setup successful");
    } else {
        crate::panic!("Failed to setup interrupts");
    }

    let (
        spi_bus,
        dc_pin,
        reset_pin,
        display_cs_pin,
        mut display_backlight_pin,
        sd_cs_pin,
        rtc_timesource,
        battery_i2c,
        mut timer,
    ) = board::get_ports(|ports| {
        (
            // Display
            ports.spi_bus.take().unwrap(),
            ports.dc_pin.take().unwrap(),
            ports.reset_pin.take().unwrap(),
            ports.display_cs_pin.take().unwrap(),
            ports.display_backlight_pin.take().unwrap(),
            // SD-Card
            ports.sd_cs_pin.take().unwrap(),
            // RTC device
            SharedRtcTimeSource(Rc::new(RtcTimeSource::new(ports.rtc_i2c.take().unwrap()))),
            // Battery device
            ports.battery_i2c.take().unwrap(),
            // Common
            ports.timer.take().unwrap(),
        )
    })
    .unwrap();

    // let (mut display, mut display_backlight_pin) = Ports::init_display(&spi_bus).unwrap();
    let exclusive_display_spi_dev = AtomicDevice::new(&spi_bus, display_cs_pin, timer).unwrap();

    let display_iface = SPIInterface::new(exclusive_display_spi_dev, dc_pin);
    let mut display = Ili9341::new(
        display_iface,
        reset_pin,
        &mut timer,
        Orientation::LandscapeFlipped,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    // Battery device
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

    // RTC
    // Set time manually:
    // let begin = NaiveDate::from_ymd_opt(2025, 8, 21)
    //     .unwrap()
    //     .and_hms_opt(15, 20, 10)
    //     .unwrap();
    // rtc.set_datetime(&begin).unwrap();

    let bg_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    let battery_bar_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::CSS_AQUA)
        .build();

    // let debug_style = PrimitiveStyleBuilder::new()
    //     .stroke_color(Rgb565::CSS_PINK)
    //     .stroke_width(1)
    //     .fill_color(Rgb565::CSS_GRAY)
    //     .build();

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

    // Turn on the display
    display_backlight_pin.set_high().unwrap();

    // SD-Card
    let exclusive_sd_spi_dev = AtomicDevice::new(&spi_bus, sd_cs_pin, timer).unwrap();
    let sdcard = SdCard::new(exclusive_sd_spi_dev, timer.clone());
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    let volume_mgr = VolumeManager::new(sdcard, rtc_timesource.clone());

    let raw_volume0 = volume_mgr.open_raw_volume(VolumeIdx(0)).unwrap();

    let root_dir = volume_mgr.open_root_dir(raw_volume0).unwrap();
    let print = |item: &DirEntry| {
        let filename = core::str::from_utf8(item.name.base_name()).unwrap();
        info!("{:?} {:?} Mb", filename, item.size as f32 / 1024.0 / 1024.0)
    };
    volume_mgr.iterate_dir(root_dir, print).unwrap();

    let shared_rtc = rtc_timesource.0;
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
            Rectangle::new(
                Point::new(0, datetime_row - row_height + 2),
                Size::new(320, row_height as u32),
            )
            .into_styled(bg_style)
            .draw(&mut display)
            .unwrap();

            text_style.set_text_color(Some(Rgb565::WHITE));
            Text::new(
                &rtc::pretty_format_datetime(dt).as_str(),
                Point::new(5, datetime_row),
                text_style,
            )
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
    }
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
