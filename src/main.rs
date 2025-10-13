#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex;
use embassy_time::Timer;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use embedded_graphics_framebuf::FrameBuf;
use {defmt_rtt as _, panic_probe as _};

// Modules
mod bus;
mod debouncer;
mod gui;
mod hid;
mod rtc;

// Main display framebuffer
const DISPLAY_WIDTH: usize = 320;
const DISPLAY_HEIGHT: usize = 240;

type FramebufferDataType = [Rgb565; DISPLAY_WIDTH * DISPLAY_HEIGHT];
type FramebufferType = mutex::Mutex<CriticalSectionRawMutex, RefCell<Option<FrameBuf<Rgb565, FramebufferDataType>>>>;
static FRAMEBUFFER: FramebufferType = mutex::Mutex::new(RefCell::new(None));

#[allow(dead_code)]
pub(crate) async fn with_framebuffer<F, R>(f: F) -> R
where
    F: FnOnce(&mut FrameBuf<Rgb565, FramebufferDataType>) -> R,
{
    let guard = crate::FRAMEBUFFER.lock().await;
    let mut cell = guard.borrow_mut();
    let fb = cell.as_mut().unwrap();
    f(fb)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize the framebuffer
    let framebuffer = FrameBuf::new(
        [Rgb565::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT],
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
    );
    // Export the framebuffer in a Mutex
    {
        let guard = FRAMEBUFFER.lock().await;
        guard.borrow_mut().replace(framebuffer);
    }

    // HID
    spawner.spawn(hid::encoder_job(p.PIO0, p.PIN_2, p.PIN_3)).unwrap();
    spawner.spawn(hid::button1_job(p.PIN_12)).unwrap();
    spawner.spawn(hid::button2_job(p.PIN_13)).unwrap();

    // Shared I2C bus
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

    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    let bg_style = PrimitiveStyleBuilder::new().fill_color(Rgb565::BLACK).build();

    {
        with_framebuffer(|fb| {
            // Display ferris
            ferris.draw(&mut *fb).unwrap();

            // Sign the thing
            Text::new("Verdigris 27.09.2025", Point::new(20, 220), style)
                .draw(&mut *fb)
                .unwrap();
        })
        .await;
    }

    loop {
        let now = timesource.pretty_print_now().await.unwrap();

        // Send the framebuffer to the screen
        with_framebuffer(|fb| {
            // Print the time
            Rectangle::new(Point::new(120, 1), Size::new(200, 20))
                .into_styled(bg_style)
                .draw(&mut *fb)
                .unwrap();
            Text::new(now.as_str(), Point::new(120, 20), style)
                .draw(&mut *fb)
                .unwrap();

            display.draw_iter(fb.into_iter()).unwrap();
        })
        .await;

        Timer::after_millis(100).await;
    }
}
