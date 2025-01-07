#![no_std]
#![no_main]

use embassy_sync::blocking_mutex::NoopMutex;
use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{mcpwm::{McPwm, PeripheralClockConfig}, prelude::*, timer, Blocking};
use esp_hal::gpio::{Level, Output};
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use log::info;
use embedded_graphics::{
    image::{Image},
    pixelcolor::BinaryColor,
    prelude::*,

};
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use esp_hal::i2c::master::{Config, I2c};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use timer::timg::TimerGroup;
use tinybmp::Bmp;

// type I2c1Bus = Mutex<NoopRawMutex, I2c<Blocking>>;

extern crate alloc;

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    //lcd setup
    // spawner.must_spawn(oled_task(peripherals.I2C0, peripherals.GPIO22, peripherals.GPIO21));
    let i2c = peripherals.I2C0;
    let mut config = Config::default();
    config.frequency = 400.kHz().into();
    let i2c = I2c::new(i2c, config).with_scl(peripherals.GPIO22).with_sda(peripherals.GPIO21);
    // let i2c_ref_cell = critical_section::Mutex::new(RefCell::new(i2c));

    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<Blocking>>>> = StaticCell::new();
    let i2c_bus
        = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);
    // let i2c_bus = CriticalSectionDevice::new(&i2c_ref_cell);
    spawner.must_spawn(oled_task(i2c_bus));



    //Motor setup
    let right_a = peripherals.GPIO17;
    let right_b = peripherals.GPIO16;

    //High or low reverses it it seems?
    Output::new(right_b, Level::High);

    let left_a = peripherals.GPIO18;
    let left_b = peripherals.GPIO19;
    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(right_a, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 5.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);
    // pin will be high 50% of the time
    pwm_pin.set_timestamp(50);
    // pin will be high 50% of the
    // TODO: Spawn some tasks
    let _ = spawner;


    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}

#[embassy_executor::task]
async fn oled_task(i2c_bus: &'static NoopMutex<RefCell<I2c<'static, Blocking>>>) {
    let i2c_device = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c_device);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    // display.init();
    let result = display.init();
    match result {
        Ok(_) => info!("Display initialized"),
        Err(e) => info!("Error initializing display: {:?}", e),
    }

    let bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("../.././assets/dvd.bmp")).unwrap();
    let mut top_left = Point::zero();
    let mut velocity = Point::new(1, 1);

    loop {

        let bottom_right = top_left + bmp.bounding_box().size;

        // Erase previous image position with a filled black rectangle
        Rectangle::with_corners(top_left, bottom_right)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut display)
            .unwrap();

        // Check if the image collided with a screen edge
        {
            if bottom_right.x > display.size().width as i32 || top_left.x < 0 {
                velocity.x = -velocity.x;
            }

            if bottom_right.y > display.size().height as i32 || top_left.y < 0 {
                velocity.y = -velocity.y;
            }
        }

        // Move the image
        top_left += velocity;

        // Draw image at new position
        Image::new(&bmp, top_left)
            .draw(&mut display.color_converted())
            .unwrap();

        // Write changes to the display
        display.flush().unwrap();
        info!("Hello world!");
        Timer::after(Duration::from_millis(50)).await;
    }
}