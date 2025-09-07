use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering},
};
use critical_section::Mutex;

use embedded_hal::digital::InputPin;
use panic_probe as _;
use rp235x_hal::{self as hal, gpio::Interrupt};

use crate::board::{Button1Pin, Button2Pin, EncoderAPin, EncoderBPin};

// Pins setup
static BUTTON1_PIN: Mutex<RefCell<Option<crate::board::Button1Pin>>> =
    Mutex::new(RefCell::new(None));
static BUTTON2_PIN: Mutex<RefCell<Option<crate::board::Button2Pin>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_A_PIN: Mutex<RefCell<Option<crate::board::EncoderAPin>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_B_PIN: Mutex<RefCell<Option<crate::board::EncoderBPin>>> =
    Mutex::new(RefCell::new(None));

// Global States, buttons
pub static BUTTON1_STATE: AtomicBool = AtomicBool::new(false);
pub static BUTTON2_STATE: AtomicBool = AtomicBool::new(false);
// Encoder
pub static ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
static ENCODER_LAST_STATE: AtomicU8 = AtomicU8::new(0);

// Debounce
static BUTTON1_LAST: AtomicU32 = AtomicU32::new(0);
static BUTTON2_LAST: AtomicU32 = AtomicU32::new(0);
static ENCODER_LAST: AtomicU32 = AtomicU32::new(0);
static BUTTON_DEBOUNCE_DELAY_US: u32 = 200_000; // in microseconds
static ENCODER_DEBOUNCE_DELAY_US: u32 = 15_000; // in microseconds

// TODO: Return a meaningful result
pub fn init(
    button1_pin: Button1Pin,
    button2_pin: Button2Pin,
    encoder_a_pin: EncoderAPin,
    encoder_b_pin: EncoderBPin,
) -> Result<(), ()> {
    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    button1_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
    button2_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
    // For encoders, both edges are relevant
    encoder_a_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
    encoder_a_pin.set_interrupt_enabled(Interrupt::EdgeHigh, true);
    encoder_b_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
    encoder_b_pin.set_interrupt_enabled(Interrupt::EdgeHigh, true);

    // // Export to global mutexes
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

    Ok(())
}

#[inline(always)]
fn now_us() -> u32 {
    // Safe because we're only *reading* from a monotonic counter
    let timer = unsafe { &*hal::pac::TIMER0::ptr() };
    timer.timerawl().read().bits() // lower 32 bits of the free-running timer
}

/// This is the interrupt handler that fires when GPIO Bank 0 detects an event
/// (like an edge).
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
            if btn1.interrupt_status(Interrupt::EdgeLow) {
                let last = BUTTON1_LAST.load(Ordering::Relaxed);
                if now_us.wrapping_sub(last) > BUTTON_DEBOUNCE_DELAY_US {
                    let cur = BUTTON1_STATE.load(Ordering::Relaxed);
                    BUTTON1_STATE.store(!cur, Ordering::Relaxed);
                    BUTTON1_LAST.store(now_us, Ordering::Relaxed);
                }
                btn1.clear_interrupt(Interrupt::EdgeLow);
            }
        }

        // BUTTON2
        if let Some(ref mut btn2) = BUTTON2_PIN.borrow(cs).borrow_mut().as_mut() {
            if btn2.interrupt_status(Interrupt::EdgeLow) {
                let last = BUTTON2_LAST.load(Ordering::Relaxed);
                if now_us.wrapping_sub(last) > BUTTON_DEBOUNCE_DELAY_US {
                    let cur = BUTTON2_STATE.load(Ordering::Relaxed);
                    BUTTON2_STATE.store(!cur, Ordering::Relaxed);
                    BUTTON2_LAST.store(now_us, Ordering::Relaxed);
                }
                btn2.clear_interrupt(Interrupt::EdgeLow);
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
                if enc_a.interrupt_status(Interrupt::EdgeLow)
                    || enc_a.interrupt_status(Interrupt::EdgeHigh)
                {
                    enc_a.clear_interrupt(Interrupt::EdgeLow);
                    enc_a.clear_interrupt(Interrupt::EdgeHigh);
                }
                if enc_a.is_high().unwrap_or(false) {
                    new_state |= 0b10;
                }
            }

            if let Some(ref mut enc_b) = ENCODER_B_PIN.borrow(cs).borrow_mut().as_mut() {
                if enc_b.interrupt_status(Interrupt::EdgeLow)
                    || enc_b.interrupt_status(Interrupt::EdgeHigh)
                {
                    enc_b.clear_interrupt(Interrupt::EdgeLow);
                    enc_b.clear_interrupt(Interrupt::EdgeHigh);
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
