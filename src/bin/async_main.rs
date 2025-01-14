#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use embedded_graphics::{image::Image, pixelcolor::BinaryColor, prelude::*};
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{GpioPin, Input, Level, Output, Pull};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::peripherals::{ADC1, ADC2, MCPWM0, MCPWM1};
use esp_hal::{
    mcpwm::{McPwm, PeripheralClockConfig},
    prelude::*,
    timer, Blocking,
};
use log::{error, info};
use mtras_sumo::io::Cursor;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use timer::timg::TimerGroup;
use tinybmp::Bmp;

extern crate alloc;

//Settings
///If set to true shows some of the more nosier logs
const DEBUG: bool = true;
const LOTS_OF_LOGS: bool = false;

//Global variables to share state between tasks

//Holds the time of flight sensor readings in mm
static RIGHT_TOF: AtomicU16 = AtomicU16::new(0);
static LEFT_TOF: AtomicU16 = AtomicU16::new(0);

//Holds the floor sensor readings
//HACK the floor sensors seem off for me. Left is always max, right is always 0 and only somewhat react to the floor change
static RIGHT_FLOOR: AtomicU16 = AtomicU16::new(0);
static LEFT_FLOOR: AtomicU16 = AtomicU16::new(0);

//Should the robot run or not
static RUN: AtomicBool = AtomicBool::new(false);

//Movement commands
//Store as numbers to make it a bit easier on static typing
static MOVEMENT: AtomicU8 = AtomicU8::new(0);

//Speed of the robot
//1-100 higher is faster. Lower may not spin the wheels at all
static SPEED: AtomicU8 = AtomicU8::new(100);

pub enum Movement {
    Stop = 0,
    Forward = 1,
    Backward = 2,
    SpinLeft = 3,
    SpinRight = 4,
}

impl Movement {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Movement::Stop,
            1 => Movement::Forward,
            2 => Movement::Backward,
            3 => Movement::SpinLeft,
            4 => Movement::SpinRight,
            _ => Movement::Stop,
        }
    }

    pub fn to_u8(&self) -> u8 {
        match self {
            Movement::Stop => 0,
            Movement::Forward => 1,
            Movement::Backward => 2,
            Movement::SpinLeft => 3,
            Movement::SpinRight => 4,
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    // This is all setup stuff for embassy and the esp32

    // peripherals is how you access pins and other peripherals on the ESP32
    // Example Input::new(peripherals.GPIO35, Pull::Up); sets gpio35 as an input with pull up for the on board button
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    info!("Embassy initialized!");

    //Sets up the i2c and creates a bus
    //The bus lets us share the i2c between tasks since Rust has different ownership rules
    let i2c = peripherals.I2C0;
    let mut config = Config::default();
    config.frequency = 400.kHz().into();
    let i2c = I2c::new(i2c, config)
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);

    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<Blocking>>>> = StaticCell::new();
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    // Spawn tasks. This is where the bulk of the application logic goes. These each run independently and asynchronous.
    // So instead of having one main loop/while there can be several with them sharing data between each

    //Quick comment above each task spawner. Can read comments in each method to see more details

    //Task to run the display
    spawner.must_spawn(oled_task(i2c_bus));
    //Task to read the time of flight sensors
    spawner.must_spawn(tof_task(i2c_bus, peripherals.GPIO32, peripherals.GPIO33));
    //Task to read the floor sensors
    spawner.must_spawn(floor_sensors_task(
        peripherals.GPIO13,
        peripherals.GPIO34,
        peripherals.ADC1,
        peripherals.ADC2,
    ));

    //Task to control the motors
    spawner.must_spawn(motors_task(
        peripherals.GPIO18,
        peripherals.GPIO19,
        peripherals.MCPWM0,
        peripherals.GPIO17,
        peripherals.GPIO16,
        peripherals.MCPWM1,
    ));

    let start_button = Input::new(peripherals.GPIO35, Pull::Up);
    let mut bot_running = false;

    loop {
        if start_button.is_low() {
            if !bot_running {
                info!("Robot starting in 3 seconds..");
                Timer::after_secs(3).await;
            }
            bot_running = !bot_running;
            match bot_running {
                true => info!("Robot starting.."),
                false => info!("Robot stopping.."),
            }
            RUN.store(bot_running, core::sync::atomic::Ordering::Relaxed);
            if bot_running {
                //Spawns the main robot loop.
                spawner.must_spawn(main_robot_loop());
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}

///This is where you write the actual logic for your robot. Does it follow a line? Avoid obstacles? etc
#[embassy_executor::task(pool_size = 1)]
async fn main_robot_loop() {
    let mut right_speed_up = 30;
    let mut left_speed_up = 30;
    loop {
        let left_tof = LEFT_TOF.load(core::sync::atomic::Ordering::Relaxed);
        let right_tof = RIGHT_TOF.load(core::sync::atomic::Ordering::Relaxed);
        let current_movement =
            Movement::from_u8(MOVEMENT.load(core::sync::atomic::Ordering::Relaxed));

        if left_tof < 45 || right_tof < 45 {
            if current_movement.to_u8() != Movement::Stop.to_u8() {
                set_movement(Movement::Stop);
                right_speed_up = 30;
                left_speed_up = 30;
            }
            Timer::after_millis(500).await;
            continue;
        }

        if (left_tof < 400 && left_tof > 20) && (right_tof < 400 && right_tof > 20) {
            set_movement(Movement::Forward);
            right_speed_up = (right_speed_up + 1).min(100);
            left_speed_up = (left_speed_up + 1).min(100);
            set_speed(right_speed_up);
        } else {
            if left_tof < 400 && left_tof > 20 {
                set_movement(Movement::SpinRight);
                set_speed(right_speed_up);
                left_speed_up = (left_speed_up - 1).min(100);
            } else if left_tof < 400 && left_tof > 20 {
                set_movement(Movement::SpinLeft);
                set_speed(left_speed_up);
                right_speed_up = (right_speed_up - 1).min(100);
            }
        }

        if left_tof > 400 && right_tof > 400 && left_speed_up > 25 && right_speed_up > 25 {
            right_speed_up = 30;
            left_speed_up = 30;
        }

        if right_speed_up >= 100 {
            right_speed_up = 100;
        }

        if left_speed_up >= 100 {
            left_speed_up = 100;
        }

        Timer::after_millis(500).await;
    }
}

///Helper function to set the speed of the robot easier
///1-100 higher is faster. Lower may not spin the wheels at all
fn set_speed(speed: u8) {
    SPEED.store(speed, core::sync::atomic::Ordering::Relaxed);
}

///Helper function to set the movement of the robot easier
fn set_movement(movement: Movement) {
    MOVEMENT.store(movement.to_u8(), core::sync::atomic::Ordering::Relaxed);
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
    let mut last_right = 0;
    let mut last_left = 0;
    loop {
        //We check the result cause I found bumps can cause the reading to fail
        let right_result = right_tof.read_range_continuous_millimeters_blocking();
        let left_result = left_tof.read_range_continuous_millimeters_blocking();

        //It appears 8,190mm is the max distance. But also seen if it is read its more than likely means the reading failed giving a false positive
        //We also check the last reading to see if it's the same as the current reading if so its likely max distance
        static MAX_DISTANCE: u16 = 8_190;
        if let Ok(right_distance) = right_result {
            if LOTS_OF_LOGS {
                info!("Right TOF: {}", right_distance);
            }
            //Safe to say if the reading is max twice it's real max distance
            if right_distance != MAX_DISTANCE || last_right != MAX_DISTANCE {
                last_right = right_distance;
                RIGHT_TOF.store(right_distance, core::sync::atomic::Ordering::Relaxed);
            }
        }

        if let Ok(left_distance) = left_result {
            if LOTS_OF_LOGS {
                info!("Left TOF: {}", left_distance);
            }
            //Safe to say if the reading is max twice it's real max distance
            if left_distance != MAX_DISTANCE || last_left != MAX_DISTANCE {
                last_left = left_distance;
                LEFT_TOF.store(left_distance, core::sync::atomic::Ordering::Relaxed);
            }
        }

        //Can tweak this to get readings faster you may need to change set_measurement_timing_budget lower
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn floor_sensors_task(left: GpioPin<13>, right: GpioPin<34>, adc1: ADC1, adc2: ADC2) {
    let mut adc1_config = AdcConfig::new();
    let mut right_pin = adc1_config.enable_pin(right, Attenuation::Attenuation0dB);
    let mut right_adc = Adc::new(adc1, adc1_config);

    let mut adc2_config = AdcConfig::new();
    let mut left_pin = adc2_config.enable_pin(left, Attenuation::Attenuation0dB);
    let mut left_adc = Adc::new(adc2, adc2_config);

    loop {
        let left_result = nb::block!(left_adc.read_oneshot(&mut left_pin));
        let right_result = nb::block!(right_adc.read_oneshot(&mut right_pin));
        match left_result {
            Ok(left_reading) => {
                LEFT_FLOOR.store(left_reading, core::sync::atomic::Ordering::Relaxed);
                if LOTS_OF_LOGS {
                    info!("Left Reading: {}", left_reading);
                }
            }
            Err(e) => error!("Error reading left floor sensor: {:?}", e),
        }
        match right_result {
            Ok(right_reading) => {
                RIGHT_FLOOR.store(right_reading, core::sync::atomic::Ordering::Relaxed);
                if LOTS_OF_LOGS {
                    info!("Right Reading: {}", right_reading);
                }
            }
            Err(e) => error!("Error reading right floor sensor: {:?}", e),
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

///Sets up the motors and controls them based on the global variables
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

    //They can all share the same config
    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 5.kHz())
        .unwrap();

    //Right motor
    let mut right_mcpwm = McPwm::new(right_mc, clock_cfg);
    right_mcpwm.operator0.set_timer(&right_mcpwm.timer0);

    let mut right_motor_a = right_mcpwm
        .operator0
        .with_pin_a(right_a, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut right_motor_b = right_mcpwm
        .operator1
        .with_pin_b(right_b, PwmPinConfig::UP_ACTIVE_HIGH);

    right_mcpwm.timer0.start(timer_clock_cfg.clone());
    right_mcpwm.timer1.start(timer_clock_cfg.clone());

    //Left motor

    let mut left_mcpwm = McPwm::new(left_mc, clock_cfg);
    left_mcpwm.operator0.set_timer(&left_mcpwm.timer0);

    let mut left_motor_a = left_mcpwm
        .operator0
        .with_pin_a(left_a, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut left_motor_b = left_mcpwm
        .operator1
        .with_pin_b(left_b, PwmPinConfig::UP_ACTIVE_HIGH);

    left_mcpwm.timer0.start(timer_clock_cfg.clone());
    left_mcpwm.timer1.start(timer_clock_cfg);

    loop {
        //check to see if the robot should run
        let current_movement = match RUN.load(core::sync::atomic::Ordering::Relaxed) {
            true => Movement::from_u8(MOVEMENT.load(core::sync::atomic::Ordering::Relaxed)),
            false => Movement::Stop,
        };
        let speed = SPEED.load(core::sync::atomic::Ordering::Relaxed) as u16;
        Movement::from_u8(MOVEMENT.load(core::sync::atomic::Ordering::Relaxed));

        //Each pin(a,b) are pwm pins. One is set to low pwm and other to selected speed. Then switch
        //to change direction

        match current_movement {
            Movement::Stop => {
                left_motor_a.set_timestamp(0);
                left_motor_b.set_timestamp(0);
                right_motor_a.set_timestamp(0);
                right_motor_b.set_timestamp(0);
            }
            Movement::Forward => {
                //Motor spins clockwise
                right_motor_a.set_timestamp(0);
                right_motor_b.set_timestamp(speed);
                //Motor spins clockwise
                left_motor_a.set_timestamp(0);
                left_motor_b.set_timestamp(speed);
            }
            Movement::Backward => {
                //Motor spins counter clockwise
                right_motor_a.set_timestamp(speed);
                right_motor_b.set_timestamp(0);
                //Motor spins counter clockwise
                left_motor_a.set_timestamp(speed);
                left_motor_b.set_timestamp(0);
            }
            Movement::SpinLeft => {
                //Motor spins  clockwise
                right_motor_a.set_timestamp(0);
                right_motor_b.set_timestamp(speed);
                //Motor spins counter clockwise
                left_motor_a.set_timestamp(speed);
                left_motor_b.set_timestamp(0);
            }
            Movement::SpinRight => {
                //Motor spins counter clockwise
                right_motor_a.set_timestamp(speed);
                right_motor_b.set_timestamp(0);
                //Motor spins clockwise
                left_motor_a.set_timestamp(0);
                left_motor_b.set_timestamp(speed);
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Task to run the oled display using embedded graphics
/// Any of these examples will work, but keep in mind the display is 128x32
/// https://github.com/embedded-graphics/examples/tree/main/eg-0.8
/// More examples from the display crate
/// https://github.com/rust-embedded-community/ssd1306/tree/master/examples
#[embassy_executor::task]
async fn oled_task(i2c_bus: &'static NoopMutex<RefCell<I2c<'static, Blocking>>>) {
    //sets up a i2c device from the bus
    let i2c_device = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c_device);

    let rotation = match DEBUG {
        true => DisplayRotation::Rotate180,
        false => DisplayRotation::Rotate0,
    };

    let mut display =
        Ssd1306::new(interface, DisplaySize128x32, rotation).into_buffered_graphics_mode();
    let result = display.init();
    match result {
        Ok(_) => info!("Display initialized"),
        Err(e) => error!("Error initializing display: {:?}", e),
    }

    //Just used for the dvb example
    let mut top_left = Point::zero();
    let mut velocity = Point::new(1, 1);
    // DVD example from https://github.com/rust-embedded-community/ssd1306/blob/master/examples/rtic_dvd.rs
    let bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("../.././assets/dvd.bmp")).unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut top_write_buffer = [0u8; 512];
    let mut top_line_cursor = Cursor::new(&mut top_write_buffer);
    let mut bottom_write_buffer = [0u8; 512];
    let mut bottom_line_cursor = Cursor::new(&mut bottom_write_buffer);

    loop {
        //If debug is enable show sensor data on the display
        if DEBUG {
            let _ = display.clear(BinaryColor::Off);
            let right_tof = RIGHT_TOF.load(core::sync::atomic::Ordering::Relaxed);
            let left_tof = LEFT_TOF.load(core::sync::atomic::Ordering::Relaxed);
            write!(top_line_cursor, "TOF: {}    TOF: {}", left_tof, right_tof).unwrap();
            let _ = Text::with_baseline(
                top_line_cursor.as_str(),
                Point::zero(),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display);

            let right_fl = RIGHT_FLOOR.load(core::sync::atomic::Ordering::Relaxed);
            let left_fl = LEFT_FLOOR.load(core::sync::atomic::Ordering::Relaxed);
            // write!(bottom_line_cursor, "L_FL: {} R_FL: {}", left_fl, right_fl).unwrap();
            let running = RUN.load(core::sync::atomic::Ordering::Relaxed);
            write!(bottom_line_cursor, "L_FL: {}  RUN: {}", left_fl, running).unwrap();
            let _ = Text::with_baseline(
                bottom_line_cursor.as_str(),
                Point::new(0, 10),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display);
            bottom_line_cursor.clear();
            top_line_cursor.clear();
        } else {
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
        }
        // Write changes to the display
        let _ = display.flush();

        Timer::after(Duration::from_millis(50)).await;
    }
}
