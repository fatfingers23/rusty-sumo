#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::{image::Image, pixelcolor::BinaryColor, prelude::*};
use esp_backtrace as _;
use esp_hal::gpio::{GpioPin, Input, Level, Output, Pull};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::peripherals::{MCPWM0, MCPWM1};
use esp_hal::{
    mcpwm::{McPwm, PeripheralClockConfig},
    prelude::*,
    timer, Blocking,
};
use log::info;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use timer::timg::TimerGroup;
use tinybmp::Bmp;

extern crate alloc;

//Settings
///If set to true shows some of the more nosier logs
const DEBUG: bool = true;

//Global variables to share state between tasks
static RIGHT_TOF: AtomicU16 = AtomicU16::new(0);
static LEFT_TOF: AtomicU16 = AtomicU16::new(0);
static RUN: AtomicBool = AtomicBool::new(false);

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
    let i2c = I2c::new(i2c, config)
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);
    // let i2c_ref_cell = critical_section::Mutex::new(RefCell::new(i2c));

    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<Blocking>>>> = StaticCell::new();
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);
    // let i2c_bus = CriticalSectionDevice::new(&i2c_ref_cell);

    // Spawn tasks
    spawner.must_spawn(oled_task(i2c_bus));
    spawner.must_spawn(tof_task(i2c_bus, peripherals.GPIO32, peripherals.GPIO33));

    spawner.must_spawn(motors_task(
        peripherals.GPIO18,
        peripherals.GPIO19,
        peripherals.MCPWM0,
        peripherals.GPIO17,
        peripherals.GPIO16,
        peripherals.MCPWM1,
    ));

    let start_button = Input::new(peripherals.GPIO35, Pull::Up);
    loop {
        if start_button.is_low() {
            Timer::after_secs(3).await;
            info!("Button pressed");
            RUN.store(true, core::sync::atomic::Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}

/// Embassy task to handle the tof sensors and write the out put to the global variables
/// uses https://github.com/copterust/vl53l0x
#[embassy_executor::task]
async fn tof_task(
    i2c_bus: &'static NoopMutex<RefCell<I2c<'static, Blocking>>>,
    right_xshut: GpioPin<32>,
    left_xshut: GpioPin<33>,
) {
    // With rust ownership we have to create two devices from the bus allowing them to be shared
    let right_i2c_device = I2cDevice::new(i2c_bus);
    let left_i2c_device = I2cDevice::new(i2c_bus);
    //Sets both tof xshut pins to low to disable for setup
    let mut right_xshut = Output::new(right_xshut, Level::Low);
    let mut left_xshut = Output::new(left_xshut, Level::Low);

    //set right xshut high to enable
    right_xshut.set_high();
    //Give some boot up time
    Timer::after_millis(100).await;

    let mut right_tof = vl53l0x::VL53L0x::new(right_i2c_device).expect("right rof i2c init failed");
    //Sets a new i2c address for the right tof sensor so we can tell them apart
    right_tof.set_address(0x52).expect("right: set address");
    right_tof
        .set_measurement_timing_budget(200_000)
        .expect("right: time budget");
    right_tof.start_continuous(0).expect("right: start");

    //Turns the left on and sets it up with the default address
    left_xshut.set_high();
    //Give some boot up time
    Timer::after_millis(100).await;

    //Sets up the left tof with the default i2c address
    let mut left_tof = vl53l0x::VL53L0x::new(left_i2c_device).expect("left tof i2c init failed");
    left_tof
        .set_measurement_timing_budget(200_000)
        .expect("left: time budget");
    left_tof.start_continuous(0).expect("left: start");

    loop {
        let right_distance = right_tof
            .read_range_continuous_millimeters_blocking()
            .expect("right: read");
        let left_distance = left_tof
            .read_range_continuous_millimeters_blocking()
            .expect("left: read");

        //It appears 8,190mm is the max distance. But also seen if it is read its more than likely means the reading failed giving a false positive
        //So this is a safe bet to stop the rest of the application from working unexpectedly
        if right_distance != 8_190 {
            RIGHT_TOF.store(right_distance, core::sync::atomic::Ordering::Relaxed);
        }
        if left_distance != 8_190 {
            LEFT_TOF.store(left_distance, core::sync::atomic::Ordering::Relaxed);
        }
        if DEBUG {
            info!("Right Distance: {}mm", right_distance);
            info!("Left Distance: {}mm", left_distance);
        }

        //Can tweak this to get readings faster, but anything under 20ms you need to change set_measurement_timing_budget lower
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn motors_task(
    left_a: GpioPin<18>,
    left_b: GpioPin<19>,
    left_mc: MCPWM0,
    right_a: GpioPin<17>,
    right_b: GpioPin<16>,
    right_mc: MCPWM1,
) {
    //Wait for start button
    while !RUN.load(core::sync::atomic::Ordering::Relaxed) {
        Timer::after(Duration::from_millis(100)).await;
    }

    //Right motor
    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut right_mcpwm = McPwm::new(right_mc, clock_cfg);

    // connect operator0 to timer0
    right_mcpwm.operator0.set_timer(&right_mcpwm.timer0);
    // connect operator0 to pin
    let mut right_motor = right_mcpwm
        .operator0
        .with_pin_a(right_a, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 5.kHz())
        .unwrap();
    right_mcpwm.timer0.start(timer_clock_cfg);

    //Left motor
    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut left_mcpwm = McPwm::new(left_mc, clock_cfg);

    // connect operator0 to timer0
    left_mcpwm.operator0.set_timer(&left_mcpwm.timer0);
    // connect operator0 to pin
    let mut left_motor = left_mcpwm
        .operator0
        .with_pin_a(left_a, PwmPinConfig::UP_ACTIVE_HIGH);
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 5.kHz())
        .unwrap();
    left_mcpwm.timer0.start(timer_clock_cfg);

    //High or low reverses it it seems?
    let mut right_direction = Output::new(right_b, Level::High);
    let mut left_direction = Output::new(left_b, Level::High);

    right_motor.set_timestamp(90);
    left_motor.set_timestamp(90);

    loop {
        let right_distance = RIGHT_TOF.load(core::sync::atomic::Ordering::Relaxed);
        // let left_distance = LEFT_TOF.load(core::sync::atomic::Ordering::Relaxed);
        if right_distance > 500 {
            right_motor.set_timestamp(25);
            left_motor.set_timestamp(25);
        } else if right_distance > 300 {
            right_motor.set_timestamp(30);
            left_motor.set_timestamp(30);
        } else if right_distance > 100 {
            right_motor.set_timestamp(90);
            left_motor.set_timestamp(90);
        } else if right_distance > 30 {
            right_motor.set_timestamp(100);
            left_motor.set_timestamp(100);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
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

        Timer::after(Duration::from_millis(50)).await;
    }
}
