#![no_std]
#![no_main]

mod bno08x_i2c;
mod fmt;

extern crate alloc;

use bno08x_i2c::Bno08xI2c;
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use embassy_stm32::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c},
    peripherals,
    time::Hertz,
    usart::{self, Config, Uart},
};
use libm::{asinf, atan2f};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use fmt::info;

bind_interrupts!(struct Irqs {
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct MotorSpeed {
    motor1: f32,
    motor2: f32,
    motor3: f32,
    motor4: f32,
}

#[repr(C)]
#[derive(Clone, Copy)]
union MotorSpeedData {
    motor_speed: MotorSpeed,
    buffer: [u8; 16],
}

fn quaternion_to_euler_angle(q: &sh2::sh2_RotationVector_t) -> (f32, f32, f32) {
    let q0 = q.real;
    let q1 = q.i;
    let q2 = q.j;
    let q3 = q.k;

    let roll = atan2f(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    let pitch = asinf(2.0 * (q0 * q2 - q3 * q1));
    let yaw = atan2f(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    (roll, pitch, yaw)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 115200;
    let mut uart = Uart::new(
        p.UART4, p.PC11, p.PC10, Irqs, p.DMA1_CH4, p.DMA1_CH2, config,
    )
    .unwrap();

    let mut config = i2c::Config::default();
    config.timeout = Duration::from_millis(10);

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(400_000),
        config,
    );

    let gpio_reset = Output::new(p.PA0, Level::High, embassy_stm32::gpio::Speed::Low);
    let gpio_int = Input::new(p.PA4, Pull::Up);
    let _ = Output::new(p.PA1, Level::High, embassy_stm32::gpio::Speed::Low);

    let mut bno08x = Bno08xI2c::new(i2c, gpio_reset, gpio_int);

    let status = bno08x.begin().await;
    info!("BNO08x status: {}", status);

    if bno08x.enable_game_rotation_vector(10 * 1000) {
        info!("Game rotation vector enabled");
    } else {
        info!("Game rotation vector enable failed");
    }

    info!("BNO08x I2C initialized");

    let mut angle_pid = pid::Pid::new(0.0, 2.0); // TODO
    angle_pid.p(10.0, 100.0);

    let mut _roll = 0.0;
    let mut _pitch = 0.0;
    let mut yaw = 0.0;

    loop {
        if bno08x.get_sensor_event() {
            let rotation_vector = bno08x.get_game_rotation_vector();
            (_roll, _pitch, yaw) = quaternion_to_euler_angle(&rotation_vector);
            info!("sensor event: yaw: {}", yaw);
        }

        let pid_output = angle_pid.next_control_output(yaw);

        // info!("T: {}, U: {}", yaw, pid_output.output);

        let motor_speed = MotorSpeed {
            motor1: pid_output.output,
            motor2: pid_output.output,
            motor3: pid_output.output,
            motor4: pid_output.output,
        };

        let motor_speed_data = MotorSpeedData { motor_speed };

        let mut buffer = [0u8; 64];

        let encoded_size = corncobs::encode_buf(unsafe { &motor_speed_data.buffer }, &mut buffer);

        match uart.write(&buffer[..encoded_size]).await {
            Ok(_) => {}
            Err(e) => {
                info!("UART write error: {:?}", e);
            }
        };

        Timer::after_millis(10).await;

        // info!("D: {}", bno08x.get_sensor_value());
    }
}
