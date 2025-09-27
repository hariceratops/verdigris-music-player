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
use alloc::{string::String};
use core::{fmt::Write, sync::atomic::Ordering};
use embedded_hal_bus::spi::AtomicDevice;

use embedded_hal::digital::OutputPin;
use embedded_sdmmc::{DirEntry, SdCard, VolumeIdx, VolumeManager};
use panic_probe as _;
use rp235x_hal::{
    // Clock,
    {self as hal, entry},
};

use libm;

// UI
// TODO: Move the Display logic in its own file
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii},
    pixelcolor::{Rgb565, RgbColor},
    prelude::{Point, Primitive, Size, *},
    primitives::{Circle, Ellipse, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Text, renderer::CharacterStyle},
};


use board::Board;
use interrupts::{BUTTON1_STATE, BUTTON2_STATE, ENCODER_COUNT};

// Battery
const MAX_BUS_VOLTAGE: u16 = 4200;
const MIN_BUS_VOLTAGE: u16 = 3800;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Rust 2024 deny it by default. Allowance is required to initialize the allocator.
#[allow(static_mut_refs)]
#[entry]
fn main() -> ! {
    info!("Verdigris Music Player start");

    //
    // Initialize hardware
    //

    // Initialize the allocator. Ensure this happen only once.
    unsafe {
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    // Get access to all hardware components
    let mut board = Board::init().expect("Failed to setup boards");

    // TODO: refactor interrupts to expose a struct instead. See if this gymnastic make sense.
    interrupts::init(
        board.take_button1_pin(),
        board.take_button2_pin(),
        board.take_encoder_a_pin(),
        board.take_encoder_b_pin(),
    )
    .expect("Failed to setup interrupts!");

    // Common object that must live and be owned in this scope
    let spi_bus = board.take_spi_bus();
    let mut timer = board.take_timer();

    // RTC
    let rtc_timesource = board.init_rtc_timesource().unwrap();
    // TODO: Provide an elegant way to set/adjust the time
    // Set time manually:
    // let begin = NaiveDate::from_ymd_opt(2025, 8, 21)
    //     .unwrap()
    //     .and_hms_opt(15, 20, 10)
    //     .unwrap();
    // rtc.set_datetime(&begin).unwrap();

    // SD-Card
    // TODO: List files in a UI, play them. Write a full spec.
    let sd_cs_pin = board.take_sd_cs_pin();
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

    // Display
    let board::Display {
        ili: mut display,
        backlight_pin: mut display_backlight_pin,
    } = board.init_display(&spi_bus, &mut timer).unwrap();

    // Battery and charge information
    // TODO: Calibrate
    let mut ina = board.init_battery();

    // Get the real time
    let shared_rtc = rtc_timesource.0;

    //
    // Main logic
    //

    let mut previous_datetime = shared_rtc.get_datetime().unwrap().and_utc();

    let bg_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    let battery_bar_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::CSS_AQUA)
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

    // Turn on the display
    display_backlight_pin.set_high().unwrap();

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
        if let Ok(ref mut ina_dev) = ina {
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

        // Display button status
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

        // Encoder value
        Rectangle::new(Point::new(160, buttons_row - 19), Size::new(160 as u32, 20))
            .into_styled(bg_style)
            .draw(&mut display)
            .unwrap();

        let encoder_value = ENCODER_COUNT.load(Ordering::Relaxed) as f32;
        let mut encoder_text: String = String::new();
        core::write!(&mut encoder_text, "Encoder: {}", encoder_value).unwrap();
        Text::new(
            encoder_text.as_str(),
            Point::new(160, buttons_row),
            text_style,
        )
        .draw(&mut display)
        .unwrap();

        let bg_style_pot = PrimitiveStyleBuilder::new()
            .fill_color(Rgb565::WHITE)
            .build();

        Circle::new(Point::new(100, buttons_row), 40)
            .into_styled(bg_style_pot)
            .draw(&mut display)
            .unwrap();

        // potentiometer center
        let pot = (120, buttons_row + 20);
        let pot_pos = libm::sincosf(-encoder_value as f32 / 10.0);
        let hand_length: f32 = 20.0; // pixels
        let right_hand = (
            pot.0 + (pot_pos.0 * hand_length) as i32,
            pot.1 + (pot_pos.1 * hand_length) as i32,
        );
        info!("{:?} {:?}", pot_pos, right_hand);

        Line::new(
            Point::new(pot.0, pot.1),
            Point::new(right_hand.0, right_hand.1),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
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
