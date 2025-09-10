
# Rationale

TODO

# Installation

```bash
rustup target install thumbv8m.main-none-eabihf
cargo install flip-link
yay -S probe-rs
cargo install --locked probe-rs-tools
cargo install cargo-generate
cargo generate --git https://github.com/rp-rs/rp235x-project-template
```


# Usage

## Hooking up the board

TODO

## Uploading

Use `cargo run` or `cargo run --release` to upload the program using the probe.

# Hardware

## Part list
- Screen:
    - doc: https://www.lcdwiki.com/2.2inch_SPI_Module_ILI9341_SKU:MSP2202
- I2S sound:
    - product: https://shop.pimoroni.com/products/pico-audio-pack?variant=32369490853971
- RTC:
    - product: https://www.waveshare.com/pico-rtc-ds3231.htm
    - doc: https://www.waveshare.com/wiki/Pico-RTC-DS3231
- Battery:
    - product: https://www.waveshare.com/pico-ups-b.htm
    - doc: https://www.waveshare.com/wiki/Pico-UPS-B

## References
- https://github.com/rp-rs/rp235x-project-template

## Additional read
- https://github.com/ariel-os/ariel-os
- https://www.raspberrypi.com/news/rust-on-rp2350/
- https://wiki.thejpster.org.uk/
- I2S example with PIO and Embassy https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/pio_i2s.rs
