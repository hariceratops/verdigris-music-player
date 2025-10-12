use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use defmt::info;
use embassy_rp::{
    Peri, bind_interrupts,
    gpio::{Input, Pull},
    peripherals::{PIN_2, PIN_3, PIN_12, PIN_13, PIO0},
    pio::{InterruptHandler as HIDInterruptHandler, Pio},
    pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
};
use embassy_time::{Duration, Instant, with_deadline};
use {defmt_rtt as _, panic_probe as _};

use crate::debouncer::Debouncer;

bind_interrupts!(struct HIDIrqs {
    PIO0_IRQ_0 => HIDInterruptHandler<PIO0>;
});

// Global States to share with the outer (crate) world
pub(crate) static BUTTON1_STATE: AtomicBool = AtomicBool::new(false);
pub(crate) static BUTTON2_STATE: AtomicBool = AtomicBool::new(false);
pub(crate) static ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);

#[embassy_executor::task]
pub async fn encoder_job(p: Peri<'static, PIO0>, a: Peri<'static, PIN_2>, b: Peri<'static, PIN_3>) {
    let Pio { mut common, sm0, .. } = Pio::new(p, HIDIrqs);

    let pio_encoder_program = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(&mut common, sm0, a, b, &pio_encoder_program);
    let mut count = 0;
    loop {
        // info!("Encoder count: {}", count);
        ENCODER_COUNT.store(count, Ordering::Relaxed);
        count += match encoder.read().await {
            Direction::Clockwise => -1,
            Direction::CounterClockwise => 1,
        };
    }
}

// FIXME: Use AnyPin. degrade is not avail on this platform, maybe there is another solution
#[embassy_executor::task]
pub async fn button1_job(pin: Peri<'static, PIN_12>) {
    let button1 = Input::new(pin, Pull::Up);
    let mut btn = Debouncer::new(button1, Duration::from_millis(100));

    loop {
        btn.debounce().await;
        let start = Instant::now();
        info!("Button 1 pressed");

        let current_state = BUTTON1_STATE.load(Ordering::Relaxed);
        BUTTON1_STATE.store(!current_state, Ordering::Relaxed);

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
pub async fn button2_job(pin: Peri<'static, PIN_13>) {
    let button2 = Input::new(pin, Pull::Up);
    let mut btn = Debouncer::new(button2, Duration::from_millis(100));
    loop {
        btn.debounce().await;
        let start = Instant::now();
        info!("Button 2 pressed");

        let current_state = BUTTON2_STATE.load(Ordering::Relaxed);
        BUTTON2_STATE.store(!current_state, Ordering::Relaxed);

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
