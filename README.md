# Rusty Sumo

This is a rust version of [mtras.com/sumo](https://mtras.com/sumo)


# Requirements
1. Install [rust](https://rustup.rs/)
2. Setup for esp dev following [3.3 of The Rust on ESP Book](https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html). *Any esp setup we are using no_std*
3. In the root of the project with the esp32 devboard connected run `cargo run --release`. This will give the bot a short little program to go forward, spin, forward, spin, forward, stop, then reverse. May want to give it a big enough area....