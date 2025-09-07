#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii},
    pixelcolor::{Rgb565, RgbColor},
    prelude::{Point, Primitive, Size},
    primitives::{Ellipse, PrimitiveStyleBuilder, Rectangle},
    text::{Text, renderer::CharacterStyle},
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::MODE_0;
use ili9341::{Ili9341, Orientation};
use panic_probe as _;
use rp235x_hal::{
    // Clock,
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::FunctionSpi,
    spi::Spi,
    {self as hal, entry},
};

// mod irqs;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use some_bsp;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
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

    let mut _encoder_a_pin = pins.gpio2.into_pull_up_input();
    let mut _encoder_b_pin = pins.gpio3.into_pull_up_input();
    let mut encoder_led_pin = pins.gpio4.into_push_pull_output();
    let mut _encoder_switch_pin = pins.gpio5.into_pull_up_input();

    let mut _button1_pin = pins.gpio12.into_pull_up_input();
    let mut button2_pin = pins.gpio13.into_pull_up_input();

    let mut backlight_pin = pins.gpio15.into_push_pull_output();
    let mosi = pins.gpio19.into_function::<FunctionSpi>();
    let miso = pins.gpio16.into_function::<FunctionSpi>();
    let sclk = pins.gpio18.into_function::<FunctionSpi>();
    let spi_pin_layout = (mosi, miso, sclk);

    let reset_pin = pins.gpio27.into_push_pull_output();
    let dc_pin = pins.gpio26.into_push_pull_output();
    let cs_pin = pins.gpio17.into_push_pull_output();

    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        MODE_0,
    );
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let excl_spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs_pin, timer).unwrap();
    let iface = SPIInterface::new(excl_spi_dev, dc_pin);

    let mut display = Ili9341::new(
        iface,
        reset_pin,
        &mut timer,
        Orientation::LandscapeFlipped,
        ili9341::DisplaySize240x320,
    )
    .unwrap();

    let bg_style = PrimitiveStyleBuilder::new()
        // .stroke_color(Rgb565::RED)
        // .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();

    Rectangle::new(Point::new(0, 0), Size::new(320, 240))
        .into_styled(bg_style)
        .draw(&mut display)
        .unwrap();

    let mut text_style = MonoTextStyle::new(&ascii::FONT_10X20, Rgb565::GREEN);

    Text::new("Verdigris", Point::new(5, 20), text_style)
        .draw(&mut display)
        .unwrap();

    text_style.set_text_color(Some(Rgb565::WHITE));
    Text::new("Music Player", Point::new(5, 45), text_style)
        .draw(&mut display)
        .unwrap();

    text_style.set_text_color(Some(Rgb565::RED));
    Text::new("Berlin, 07.2025", Point::new(5, 70), text_style)
        .draw(&mut display)
        .unwrap();

    let mut x_pos = 0;
    let mut y_pos = 120;
    let dot_size = 10;

    let dot_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
        .build();

    backlight_pin.set_high().unwrap();
    encoder_led_pin.set_high().unwrap();
    loop {
        info!("loop");
        timer.delay_ms(100);
        if button2_pin.is_high().unwrap() {
            encoder_led_pin.set_high().unwrap();
        } else {
            encoder_led_pin.set_low().unwrap();
        }

        Ellipse::new(Point::new(x_pos, y_pos), Size::new(dot_size, dot_size))
            .into_styled(dot_style)
            .draw(&mut display)
            .unwrap();
        x_pos += dot_size as i32;
        if x_pos >= 320 {
            x_pos = 0;
            y_pos += dot_size as i32;
        }
        if y_pos >= 240 {
            y_pos = 120;
            Rectangle::new(Point::new(0, 120), Size::new(320, 120))
                .into_styled(bg_style)
                .draw(&mut display)
                .unwrap();
        }
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 Template"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];
