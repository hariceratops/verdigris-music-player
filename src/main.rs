#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_graphics::prelude::WebColors;

use core::fmt::Write;
use heapless::String;
//

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use embedded_hal::spi::MODE_0;
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

// Interrupts
use core::cell::RefCell;
use critical_section::Mutex;

// Peripherals
use display_interface_spi::SPIInterface;
use ds323x::{DateTimeAccess, Datelike, Ds323x, NaiveDate, Timelike};
use ina219::SyncIna219;
use ina219::address::Address;

// UI
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii},
    pixelcolor::{Rgb565, RgbColor},
    prelude::{Point, Primitive, Size},
    primitives::{Ellipse, PrimitiveStyleBuilder, Rectangle},
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

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = Pin<gpio::bank0::Gpio4, gpio::FunctionSioOutput, gpio::PullDown>;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)
type ButtonPin = Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedAndButton = (LedPin, ButtonPin);

/// This how we transfer our Led and Button pins into the Interrupt Handler.
/// We'll have the option hold both using the LedAndButton type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_STATE: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");

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

    // Mapping
    // let mut encoder_a_pin = pins.gpio2.into_pull_up_input();
    // let mut encoder_b_pin = pins.gpio3.into_pull_up_input();
    // let mut encoder_led_pin = pins.gpio4.into_push_pull_output();
    // let mut encoder_switch_pin = pins.gpio5.into_pull_up_input();
    //
    // let mut button1_pin = pins.gpio12.into_pull_up_input();
    // let mut button2_pin = pins.gpio13.into_pull_up_input();


    //
    // Interrupts
    //

    // Configure GPIO 4 as an output to drive our LED.
    // we can use reconfigure() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    // let led = pins.gpio25.reconfigure();
    // let encoder_led_pin = pins.gpio4.into_push_pull_output();
    let encoder_led_pin = pins.gpio4.reconfigure();

    // Set up the GPIO pin that will be our input
    // let in_pin = pins.gpio26.reconfigure();
    // let button1_pin = pins.gpio12.into_pull_up_input();
    let button1_pin = pins.gpio12.reconfigure();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    button1_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_STATE
            .borrow(cs)
            .replace(Some((encoder_led_pin, button1_pin)));
    });

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
    let mut rtc = Ds323x::new_ds3231(rtc_i2c);
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
        iface,
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

    let button_dot_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
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

    let mut previous_datetime = rtc.datetime().unwrap().and_utc();

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
        let datetime = rtc.datetime().unwrap().and_utc();
        if previous_datetime.timestamp() % 100 != datetime.timestamp() % 100 {
            let mut line: String<20> = String::new();
            let dt = rtc.datetime().unwrap();
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

            Rectangle::new(Point::new(0, datetime_row - row_height + 2), Size::new(320, row_height as u32))
                .into_styled(bg_style)
                .draw(&mut display)
                .unwrap();

            text_style.set_text_color(Some(Rgb565::WHITE));
            Text::new(line.as_str(), Point::new(5, datetime_row), text_style)
                .draw(&mut display)
                .unwrap();

            previous_datetime = datetime;
        }

        // Get and display the battery status
        if let Some(ref mut ina_dev) = ina {
            Rectangle::new(Point::new(0, battery_row - row_height + 2), Size::new(320, row_height as u32))
                .into_styled(bg_style)
                .draw(&mut display)
                .unwrap();

            let shunt = ina_dev.shunt_voltage().unwrap();
            let bus = ina_dev.bus_voltage().unwrap();
            let bar_resolution = (MAX_BUS_VOLTAGE - MIN_BUS_VOLTAGE) as f32;
            let bar_percentage = 100.0 / bar_resolution * (bus.voltage_mv() - MIN_BUS_VOLTAGE) as f32;
            // This averages a bit the percentage and prevents flickering
            let bar_width = bar_percentage * 3.2;

            if shunt.shunt_voltage_mv() < 0 {
                text_style.set_text_color(Some(Rgb565::RED));
            } else {
                text_style.set_text_color(Some(Rgb565::GREEN));
            }

            let mut percentage_line: String<5> = String::new();
            core::write!(&mut percentage_line, "{}%", bar_percentage).unwrap();
            // info!("{}", bus_line.as_str());
            Text::new(percentage_line.as_str(), Point::new(5, battery_row), text_style)
                .draw(&mut display)
                .unwrap();

            let mut bus_line: String<20> = String::new();
            core::write!(&mut bus_line, "Bus: {} mV", bus.voltage_mv()).unwrap();
            // info!("{}", bus_line.as_str());
            Text::new(bus_line.as_str(), Point::new(50, battery_row), text_style)
                .draw(&mut display)
                .unwrap();

            let mut shunt_line: String<20> = String::new();
            core::write!(&mut shunt_line, "Shunt: {} mV", shunt.shunt_voltage_mv()).unwrap();
            // info!("{}", shunt_line.as_str());
            Text::new(
                shunt_line.as_str(),
                Point::new(185, battery_row),
                text_style,
            )
            .draw(&mut display)
            .unwrap();

            Rectangle::new(Point::new(0, battery_bar_row - (row_height / 2) + 2), Size::new(320 as u32, 2))
                .into_styled(bg_style)
                .draw(&mut display)
                .unwrap();
            Rectangle::new(Point::new(0, battery_bar_row - (row_height / 2) + 2), Size::new(bar_width as u32, 2))
                .into_styled(battery_bar_style)
                .draw(&mut display)
                .unwrap();
        } else {
            text_style.set_text_color(Some(Rgb565::RED));
            Text::new("No battery", Point::new(5, battery_row), text_style)
                .draw(&mut display)
                .unwrap();
        }

        // info!("loop");
        timer.delay_ms(100);

        Ellipse::new(Point::new(buttons_row, 5), Size::new(button_dot_size, button_dot_size))
            .into_styled(button_dot_style)
            .draw(&mut display)
            .unwrap();

        Ellipse::new(Point::new(buttons_row, 50), Size::new(button_dot_size, button_dot_size))
            .into_styled(button_dot_style)
            .draw(&mut display)
            .unwrap();

        // x_pos += dot_size as i32;
        // if x_pos >= 320 {
        //     x_pos = 0;
        //     y_pos += dot_size as i32;
        // }
        // if y_pos >= 240 {
        //     y_pos = 120;
        //     Rectangle::new(Point::new(0, 120), Size::new(320, 120))
        //         .into_styled(bg_style)
        //         .draw(&mut display)
        //         .unwrap();
        // }
    }
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
        // Grab a mutable reference to the global state, using the CS token as
        // proof we have turned off interrupts. Performs a run-time borrow check
        // of the RefCell to ensure no-one else is currently borrowing it (and
        // they shouldn't, because we're in a critical section right now).
        let mut maybe_state = GLOBAL_STATE.borrow_ref_mut(cs);
        // Need to check if our Option<LedAndButtonPins> contains our pins
        if let Some((led, button)) = maybe_state.as_mut() {
            // Check if the interrupt source is from the pushbutton going from high-to-low.
            // Note: this will always be true in this example, as that is the only enabled GPIO interrupt source
            if button.interrupt_status(gpio::Interrupt::EdgeLow) {
                // toggle can't fail, but the embedded-hal traits always allow for it
                // we can discard the return value by assigning it to an unnamed variable
                let _ = led.toggle();
                // Our interrupt doesn't clear itself.
                // Do that now so we don't immediately jump back to this interrupt handler.
                button.clear_interrupt(gpio::Interrupt::EdgeLow);
            }
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
