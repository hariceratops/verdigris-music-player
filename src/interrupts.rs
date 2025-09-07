// Alias for our HAL crate
use rp235x_hal::{
    gpio,
    {self as hal},
};


// Some traits we need
use embedded_hal::digital::StatefulOutputPin;
// Some more helpful aliases
use core::cell::RefCell;
use critical_section::Mutex;

#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioOutput, gpio::PullDown>;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)
type ButtonPin = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedAndButton = (LedPin, ButtonPin);

/// This how we transfer our Led and Button pins into the Interrupt Handler.
/// We'll have the option hold both using the LedAndButton type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_STATE: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));

pub fn setup(pins: &mut hal::gpio::Pins) {
    // let mut encoder_a_pin = pins.gpio2.into_pull_up_input();
    // let mut encoder_b_pin = pins.gpio3.into_pull_up_input();
    // let mut encoder_led_pin = pins.gpio4.into_push_pull_output();
    // let mut encoder_switch_pin = pins.gpio5.into_pull_up_input();
    //
    // let mut button1_pin = pins.gpio12.into_pull_up_input();
    // let mut button2_pin = pins.gpio13.into_pull_up_input();

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
}

pub fn worker() {
    hal::arch::wfi();
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
