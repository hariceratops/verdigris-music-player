use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Ellipse, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Text, renderer::CharacterStyle},
};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

pub(crate) async fn button(state: bool, pos: (i32, i32)) {
    let button_style = if state {
        PrimitiveStyleBuilder::new().fill_color(Rgb565::BLUE).build()
    } else {
        PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build()
    };

    crate::with_framebuffer(|fb| {
        Ellipse::new(Point::new(pos.0, pos.1), Size::new(25, 25))
            .into_styled(button_style)
            .draw(&mut *fb)
            .unwrap();
    })
    .await;
}

pub(crate) async fn encoder(count: i32) {
    let row = 120;
    let bg_style = PrimitiveStyleBuilder::new().fill_color(Rgb565::BLACK).build();
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);

    crate::with_framebuffer(|fb| {
        // Encoder value
        Rectangle::new(Point::new(100, row - 19), Size::new(220 as u32, 20))
            .into_styled(bg_style)
            .draw(&mut *fb)
            .unwrap();

        let mut encoder_text: String<20> = String::new();
        write!(&mut encoder_text, "Encoder: {}", count).unwrap();
        Text::new(encoder_text.as_str(), Point::new(160, row), text_style)
            .draw(&mut *fb)
            .unwrap();

        let bg_style_pot = PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build();

        Circle::new(Point::new(100, row), 40)
            .into_styled(bg_style_pot)
            .draw(&mut *fb)
            .unwrap();

        // potentiometer center
        let radius = 20.0;
        let pot = (120, row + (radius as i32));
        let pot_pos = libm::sincosf(-count as f32 / 10.0);
        let hand_length: f32 = radius; // pixels
        let right_hand = (
            pot.0 + (pot_pos.0 * hand_length) as i32,
            pot.1 + (pot_pos.1 * hand_length) as i32,
        );

        Line::new(Point::new(pot.0, pot.1), Point::new(right_hand.0, right_hand.1))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
            .draw(&mut *fb)
            .unwrap();
    })
    .await;
}

pub(crate) async fn charge(shunt_voltage: i16, bus_voltage: u16) {
    let row = 90;
    let row_height = 20;
    let bg_style = PrimitiveStyleBuilder::new().fill_color(Rgb565::BLACK).build();
    let mut text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    let battery_bar_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::CSS_AQUA)
        .build();

    crate::with_framebuffer(|fb| {
        Rectangle::new(Point::new(0, row - row_height + 2), Size::new(320, (row_height + 5) as u32))
            .into_styled(bg_style)
            .draw(&mut *fb)
            .unwrap();

        const MAX_BUS_VOLTAGE: u16 = 4200;
        const MIN_BUS_VOLTAGE: u16 = 3800;

        let bar_resolution = (MAX_BUS_VOLTAGE - MIN_BUS_VOLTAGE) as f32;
        let mut bar_percentage = 100.0 / bar_resolution * (bus_voltage - MIN_BUS_VOLTAGE) as f32;
        if bar_percentage < 0.0 { bar_percentage = 0.0 }
        if bar_percentage > 100.0 { bar_percentage = 100.0 }
        // This averages a bit the percentage and prevents flickering
        let bar_width = bar_percentage * 3.2;

        if shunt_voltage < 0 {
            text_style.set_text_color(Some(Rgb565::RED));
        } else {
            text_style.set_text_color(Some(Rgb565::GREEN));
        }

        let mut percentage_line: String<15> = String::new();
        write!(&mut percentage_line, "{}%", bar_percentage).unwrap();
        Text::new(percentage_line.as_str(), Point::new(5, row), text_style)
            .draw(&mut *fb)
            .unwrap();

        let mut bus_line: String<15> = String::new();
        write!(&mut bus_line, "Bus: {} mV", bus_voltage).unwrap();
        // info!("{}", bus_line.as_str());
        Text::new(bus_line.as_str(), Point::new(50, row), text_style)
            .draw(&mut *fb)
            .unwrap();

        let mut shunt_line: String<15> = String::new();
        write!(&mut shunt_line, "Shunt: {} mV", shunt_voltage).unwrap();
        // info!("{}", shunt_line.as_str());
        Text::new(shunt_line.as_str(), Point::new(185, row), text_style)
            .draw(&mut *fb)
            .unwrap();

        Rectangle::new(
            Point::new(0, row + 5),
            Size::new(320 as u32, 2),
        )
        .into_styled(bg_style)
        .draw(&mut *fb)
        .unwrap();
        Rectangle::new(
            Point::new(0, row + 5),
            Size::new(bar_width as u32, 2),
        )
        .into_styled(battery_bar_style)
        .draw(&mut *fb)
        .unwrap();
    })
    .await;
}
