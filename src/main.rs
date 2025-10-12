#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex;
use embassy_time::Timer;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    pixelcolor::Rgb565,
    primitives::{Circle, Ellipse, Line, PrimitiveStyle},
};
use embedded_graphics_framebuf::FrameBuf;
use heapless::String;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

// Modules
mod bus;
mod debouncer;
mod hid;
mod rtc;

// Main display framebuffer
const DISPLAY_WIDTH: usize = 320;
const DISPLAY_HEIGHT: usize = 240;

type FramebufferType = [Rgb565; DISPLAY_WIDTH * DISPLAY_HEIGHT];
static FRAMEBUFFER_DATA: StaticCell<FramebufferType> = StaticCell::new();
static FRAMEBUFFER: StaticCell<mutex::Mutex<CriticalSectionRawMutex, FrameBuf<Rgb565, FramebufferType>>> =
    StaticCell::new();
static FRAMEBUFFER_DIRTY: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize the framebuffer
    // let data = FRAMEBUFFER_DATA.init_with(|| [Rgb565::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT]);
    // static mut DATA: [Rgb565; DISPLAY_WIDTH * DISPLAY_HEIGHT] = [Rgb565::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT];
    // let data: [Rgb565; DISPLAY_WIDTH * DISPLAY_HEIGHT] = ;
    let mut framebuffer = FrameBuf::new([Rgb565::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT], DISPLAY_WIDTH, DISPLAY_HEIGHT);
    let shared_framebuffer = FRAMEBUFFER.init(mutex::Mutex::new(framebuffer));

    // HID
    spawner.spawn(hid::encoder_job(p.PIO0, p.PIN_2, p.PIN_3)).unwrap();
    spawner.spawn(hid::button1_job(p.PIN_12)).unwrap();
    spawner.spawn(hid::button2_job(p.PIN_13)).unwrap();

    // Shared I2C bus
    // DOC: https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/shared_bus.rs
    let timesource = bus::init_rtc(p.I2C0, p.PIN_21, p.PIN_20);
    // timesource.set_datetime_from_iso8601("2025-10-12 11:31:00").await;

    let ina = bus::init_battery(p.I2C1, p.PIN_7, p.PIN_6).await.unwrap();
    spawner.spawn(bus::battery_job(ina)).unwrap();

    // Shared SPI bus
    let shared_spi_bus = bus::init_shared_spi(p.SPI0, p.PIN_16, p.PIN_19, p.PIN_18);
    let mut display = bus::init_display(shared_spi_bus, p.PIN_27, p.PIN_17, p.PIN_26).unwrap();

    let backlight_pin = p.PIN_15;
    let mut backlight = Output::new(backlight_pin, Level::Low);

    display.clear(Rgb565::WHITE).unwrap();

    backlight.set_high();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(5, 5));

    // Display the image
    ferris.draw(&mut display).unwrap();

    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    let bg_style = PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build();

    Text::new("Verdigris 27.09.2025", Point::new(20, 220), style)
        .draw(&mut display)
        .unwrap();

    // let mut previous_datetime = timesource.get_datetime().await.unwrap().and_utc();

    loop {
        // previous_datetime = timesource.get_datetime().await.unwrap().and_utc();

        // //
        // // Print the time
        // //
        // let now = timesource.pretty_print_now().await.unwrap();
        // Rectangle::new(Point::new(120, 1), Size::new(200, 20))
        //     .into_styled(bg_style)
        //     .draw(&mut display)
        //     .unwrap();
        // Text::new(now.as_str(), Point::new(120, 20), style)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // //
        // // Print buttons and encoder status
        // //
        // let buttons_row = 80;
        //
        // // Button 1
        // let button1_style = if hid::BUTTON1_STATE.load(Ordering::Relaxed) {
        //     PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build()
        // } else {
        //     PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build()
        // };
        //
        // Ellipse::new(Point::new(5, buttons_row), Size::new(25, 25))
        //     .into_styled(button1_style)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // // Button 2
        // let button2_style = if hid::BUTTON2_STATE.load(Ordering::Relaxed) {
        //     PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build()
        // } else {
        //     PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build()
        // };
        //
        // Ellipse::new(Point::new(50, buttons_row), Size::new(25, 25))
        //     .into_styled(button2_style)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // // Encoder value
        // Rectangle::new(Point::new(100, buttons_row - 19), Size::new(220 as u32, 20))
        //     .into_styled(bg_style)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // let encoder_value = hid::ENCODER_COUNT.load(Ordering::Relaxed) as f32;
        // let mut encoder_text: String<20> = String::new();
        // write!(&mut encoder_text, "Encoder: {}", encoder_value).unwrap();
        // Text::new(encoder_text.as_str(), Point::new(160, buttons_row), style)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // let bg_style_pot = PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build();
        //
        // Circle::new(Point::new(100, buttons_row), 40)
        //     .into_styled(bg_style_pot)
        //     .draw(&mut display)
        //     .unwrap();
        //
        // // potentiometer center
        // let radius = 20.0;
        // let pot = (120, buttons_row + (radius as i32));
        // let pot_pos = libm::sincosf(-encoder_value as f32 / 10.0);
        // let hand_length: f32 = radius; // pixels
        // let right_hand = (
        //     pot.0 + (pot_pos.0 * hand_length) as i32,
        //     pot.1 + (pot_pos.1 * hand_length) as i32,
        // );
        //
        // Line::new(Point::new(pot.0, pot.1), Point::new(right_hand.0, right_hand.1))
        //     .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
        //     .draw(&mut display)
        //     .unwrap();

        {
            let guard = shared_framebuffer.lock().await;
            display.draw_iter(guard.into_iter()).unwrap();
            // let area = Rectangle::new(Point::new(0, 0), guard.size());
            // display.fill_contiguous(&area, unsafe { DATA }).unwrap();
        }

        Timer::after_millis(100).await;
    }
}
