use defmt::info;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder};
use embassy_time::{Duration, Instant, with_deadline};
use {defmt_rtt as _, panic_probe as _};

use crate::debouncer::Debouncer;

#[embassy_executor::task]
pub async fn encoder_job(mut encoder: PioEncoder<'static, PIO0, 0>) {
    let mut count = 0;
    loop {
        info!("Encoder count: {}", count);
        count += match encoder.read().await {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
    }
}

#[embassy_executor::task]
pub async fn button1_job(mut btn: Debouncer<'static>) {
    loop {
        btn.debounce().await;
        let start = Instant::now();
        info!("Button 1 pressed");

        match with_deadline(start + Duration::from_secs(1), btn.debounce()).await {
            // Button Released < 1s
            Ok(_) => {
                info!("Button pressed for: {}ms", start.elapsed().as_millis());
                continue;
            }
            // button held for > 1s
            Err(_) => {
                info!("Button Held for more than 1 second");
            }
        }
    }
}

#[embassy_executor::task]
pub async fn button2_job(mut btn: Debouncer<'static>) {
    loop {
        btn.debounce().await;
        let start = Instant::now();
        info!("Button 2 pressed");

        match with_deadline(start + Duration::from_secs(1), btn.debounce()).await {
            // Button Released < 1s
            Ok(_) => {
                info!("Button pressed for: {}ms", start.elapsed().as_millis());
                continue;
            }
            // button held for > 1s
            Err(_) => {
                info!("Button Held for more than 1 second");
            }
        }
    }
}
