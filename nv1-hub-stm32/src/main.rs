#![no_std]
#![no_main]

mod bno08x_i2c;
mod fmt;
mod jetson_msgpack;
mod omni;

extern crate alloc;

use core::borrow::Borrow;

use bno08x_i2c::Bno08xI2c;
use defmt::error;
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use bbqueue::BBBuffer;
use embassy_stm32::{
    bind_interrupts,
    dma::NoDma,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c},
    pac::common::W,
    peripherals,
    time::Hertz,
    usart::{self, Config, Uart},
};
use embedded_graphics::Drawable;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Dimensions, Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle, StyledDrawable},
};
use jetson_msgpack::JetsonMsgPack;
use libm::{asinf, atan2f};
use nv1_hub_ui::{HubUi, HubUiEvent, HubUiOptions};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use serde::Serialize;
use ssd1306::{mode::DisplayConfig, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use fmt::info;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
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
union MdData {
    motor_speed: MotorSpeed,
    buffer: [u8; 16],
}

static BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();

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

    let mut uart3_config = Config::default();
    uart3_config.baudrate = 115200;
    let mut uart_jetson = Uart::new(
        p.USART3,
        p.PC5,
        p.PB10,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH1,
        uart3_config,
    )
    .unwrap();

    let mut uart4_config = Config::default();
    uart4_config.baudrate = 115200;
    let mut uart_md = Uart::new(
        p.UART4,
        p.PC11,
        p.PC10,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH2,
        uart4_config,
    )
    .unwrap();

    let mut uart6_config = Config::default();
    uart6_config.baudrate = bno08x_rvc::BNO08X_UART_RVC_BAUD_RATE;
    let mut uart_bno = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        Irqs,
        p.DMA2_CH6,
        p.DMA2_CH1,
        uart6_config,
    )
    .unwrap();

    let msgpack_default = corepack::to_bytes(JetsonMsgPack::default()).unwrap();
    let cobs_decoded_data_size: usize = msgpack_default.len();

    let mut config = i2c::Config::default();
    config.timeout = Duration::from_millis(100);

    let ssd1306_i2c = I2c::new(
        p.I2C3,
        p.PA8,
        p.PC9,
        Irqs,
        NoDma,
        NoDma,
        Hertz::khz(400),
        config,
    );

    let ssd1306_interface = I2CDisplayInterface::new(ssd1306_i2c);
    let mut ssd1306 = Ssd1306::new(
        ssd1306_interface,
        DisplaySize128x64,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    ssd1306.init().unwrap();

    let ui_options = HubUiOptions {
        display_size: Size::new(128, 64),
        menu_vertical_num: 4,
    };
    let mut ui = HubUi::new(&mut ssd1306, ui_options);
    let _ = ui.update(&HubUiEvent::None);

    ssd1306.flush().unwrap();
    let mut gpio_reset = Output::new(p.PA0, Level::High, embassy_stm32::gpio::Speed::Low);
    gpio_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    gpio_reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

    let (mut proc, mut parser) = match bno08x_rvc::create(BB.borrow()) {
        Ok((proc, pars)) => (proc, pars),
        Err(_e) => {
            error!("Can't create bno08x-rvc");
            loop {}
        }
    };

    let mut _roll = 0.0;
    let mut _pitch = 0.0;
    let mut yaw = 0.0;

    let mut angle_pid = pid::Pid::new(0.0, 100.0); // TODO
    angle_pid.p(3.0, 100.0);

    const WHEEL_R: f32 = 1.0;
    const THREAD: f32 = 1.0;
    let wheel_calc1 = omni::OmniWheel::new(45.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc2 = omni::OmniWheel::new(135.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc3 = omni::OmniWheel::new(225.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc4 = omni::OmniWheel::new(315.0_f32.to_radians(), WHEEL_R, THREAD);

    let mut motor_speed = MotorSpeed {
        motor1: 0.0,
        motor2: 0.0,
        motor3: 0.0,
        motor4: 0.0,
    };

    info!("nv1-hub initialized");

    loop {
        info!("Instant: {:?}", embassy_time::Instant::now().as_millis());
        let mut buf = [0u8; 19];
        uart_bno.read(&mut buf).await.unwrap();
        proc.process_slice(&buf).unwrap();
        parser
            .worker(|frame| {
                yaw = (frame.as_pretty_frame().yaw as f32).to_radians();
                info!("Yaw: {}", yaw);
            })
            .unwrap();

        let pid_output = angle_pid.next_control_output(yaw);

        // info!("T: {}, U: {}", yaw, pid_output.output);

        let motor_speed = MotorSpeed {
            motor1: -pid_output.output,
            motor2: -pid_output.output,
            motor3: -pid_output.output,
            motor4: -pid_output.output,
        };

        let motor_speed_data = MdData { motor_speed };

        let mut buffer = [0u8; 64];
        let encoded_size = corncobs::encode_buf(unsafe { &motor_speed_data.buffer }, &mut buffer);

        match uart_md.write(&buffer[..encoded_size]).await {
            Ok(_) => {}
            Err(e) => {
                info!("UART write error: {:?}", e);
            }
        };

        // let mut buf = [0u8; 64];
        // match uart_jetson.read(&mut buf).await {
        //     Ok(_) => {
        //         let mut cobs_decoded_buf = [0; 64];
        //         match corncobs::decode_buf(&buf, &mut cobs_decoded_buf) {
        //             Ok(size) => {
        //                 if size != cobs_decoded_data_size {
        //                     info!("Invalid data size: {}", size);
        //                     continue;
        //                 }

        //                 let msgpack_buf = corepack::from_bytes::<JetsonMsgPack>(
        //                     &cobs_decoded_buf[0..cobs_decoded_data_size],
        //                 )
        //                 .unwrap();

        //                 info!("Linear X: {}", msgpack_buf.cmd_vel.linear_x);
        //                 info!("Linear Y: {}", msgpack_buf.cmd_vel.linear_y);
        //                 info!("Angular Z: {}", msgpack_buf.cmd_vel.angular_z);

        //                 angle_pid.setpoint(msgpack_buf.cmd_vel.angular_z);

        //                 let mut motor1 = 0.0;
        //                 let mut motor2 = 0.0;
        //                 let mut motor3 = 0.0;
        //                 let mut motor4 = 0.0;

        //                 [wheel_calc1, wheel_calc2, wheel_calc3, wheel_calc4]
        //                     .iter()
        //                     .zip([motor1, motor2, motor3, motor4].iter_mut())
        //                     .for_each(|(wheel, motor)| {
        //                         *motor = wheel.calculate(
        //                             msgpack_buf.cmd_vel.linear_x,
        //                             msgpack_buf.cmd_vel.linear_y,
        //                             msgpack_buf.cmd_vel.angular_z,
        //                         );
        //                     });

        //                 motor_speed = MotorSpeed {
        //                     motor1,
        //                     motor2,
        //                     motor3,
        //                     motor4,
        //                 };
        //             }
        //             Err(err) => {
        //                 info!("Failed to decode data");
        //                 info!("Data: {:?}", buf);
        //                 match err {
        //                     corncobs::CobsError::Truncated => info!("Truncated"),
        //                     corncobs::CobsError::Corrupt => info!("Corrupt"),
        //                 }

        //                 // read until 0
        //                 let mut buffer = [0; 1];
        //                 loop {
        //                     uart_md.read(&mut buffer).await.unwrap();
        //                     if buffer[0] == 0 {
        //                         break;
        //                     }
        //                 }
        //             }
        //         }
        //     }
        //     Err(err) => {
        //         if err == usart::Error::Overrun {
        //             info!("Overrun");
        //             continue;
        //         } else {
        //             info!("Failed to read data");
        //             info!("Error: {:?}", err);
        //             continue;
        //         }
        //     }
        // };

        // Timer::after_millis(10).await;

        // info!("D: {}", bno08x.get_sensor_value());
    }
}
