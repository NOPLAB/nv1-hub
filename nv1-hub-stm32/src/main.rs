#![no_std]
#![no_main]

mod fmt;
mod omni;

extern crate alloc;

use core::{borrow::Borrow, cell::RefCell};

use alloc::vec::Vec;
use defmt::error;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use bbqueue::BBBuffer;
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    dma::NoDma,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c},
    peripherals,
    time::Hertz,
    usart::{self, Config, Uart},
};
use embassy_time::Delay;
use embedded_graphics::prelude::Size;
use libm::{asinf, atan2f, cosf, powf, sinf, sqrtf};
use num_traits::{AsPrimitive, Num};
use nv1_hub_ui::{HubUi, HubUiEvent, HubUiOptions};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
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

static G_MSG_PACK_RX: Mutex<ThreadModeRawMutex, RefCell<nv1_msg::HubMsgPackRx>> =
    Mutex::new(RefCell::new(nv1_msg::HubMsgPackRx {
        vel: nv1_msg::Velocity {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z: 0.0,
        },
        kick: false,
    }));

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

fn generate_adc_vec<T>(sin: &mut [T], cos: &mut [T], offset: f32, one_angle: f32, mul: f32)
where
    f32: AsPrimitive<T>,
    T: Num + Copy + 'static,
{
    for i in 0..sin.len() {
        sin[i] = (sinf(i as f32 * one_angle + offset) * mul).as_();
        cos[i] = (cosf(i as f32 * one_angle + offset) * mul).as_();
    }
}

fn calculate_adc_vec<T>(adc: &[T], adc_sin: &[T], adc_cos: &[T], mul: T) -> (f32, f32)
where
    T: Num + Copy + 'static + AsPrimitive<f32>,
{
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;

    for i in 0..adc.len() {
        sum_x = sum_x + (adc_cos[i] * adc[i]).as_();
        sum_y = sum_y + (adc_sin[i] * adc[i]).as_();
    }

    let norm = sqrtf(powf(sum_x / adc.len() as f32, 2.0) + powf(sum_y / adc.len() as f32, 2.0));

    (
        sum_x / adc.len() as f32 / norm,
        sum_y / adc.len() as f32 / norm,
    )
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut p = embassy_stm32::init(Default::default());

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

    let mut delay = Delay;
    let mut adc1 = Adc::new(p.ADC1, &mut delay);
    adc1.set_sample_time(embassy_stm32::adc::SampleTime::Cycles3);

    let mut line_s0 = Output::new(p.PB12, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut line_s1 = Output::new(p.PB13, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut line_s2 = Output::new(p.PB14, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut line_s3 = Output::new(p.PB15, Level::Low, embassy_stm32::gpio::Speed::High);

    let mut ir_s0 = Output::new(p.PB0, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut ir_s1 = Output::new(p.PB1, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut ir_s2 = Output::new(p.PB4, Level::Low, embassy_stm32::gpio::Speed::High);
    let mut ir_s3 = Output::new(p.PB5, Level::Low, embassy_stm32::gpio::Speed::High);

    let mut adc_line_sin = [0.0f32; 32];
    let mut adc_line_cos = [0.0f32; 32];

    generate_adc_vec(
        &mut adc_line_sin,
        &mut adc_line_cos,
        (90 as f32).to_radians(),
        -((360.0 / 32.0) as f32).to_radians(),
        1.0,
    );

    info!("ADC Line Sin: {:?}", adc_line_sin);

    let mut adc_ir_sin = [0.0f32; 16];
    let mut adc_ir_cos = [0.0f32; 16];

    generate_adc_vec(
        &mut adc_ir_sin,
        &mut adc_ir_cos,
        (90 as f32).to_radians(),
        -((360.0 / 16.0) as f32).to_radians(),
        1.0,
    );

    let mut gpio_ui_toggle = Input::new(p.PC12, Pull::None);

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
    angle_pid.p(1.2, 100.0);

    const WHEEL_R: f32 = 25.0 / 1000.0;
    const THREAD: f32 = 108.0 / 1000.0;
    let wheel_calc1 = omni::OmniWheel::new(135.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc2 = omni::OmniWheel::new(225.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc3 = omni::OmniWheel::new(315.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc4 = omni::OmniWheel::new(45.0_f32.to_radians(), WHEEL_R, THREAD);

    let mut motor_speed;

    spawner.spawn(uart_task(uart_jetson)).unwrap();

    info!("nv1-hub initialized");

    loop {
        let mut buf = [0u8; 19];
        uart_bno.read(&mut buf).await.unwrap();
        proc.process_slice(&buf).unwrap();
        parser
            .worker(|frame| {
                yaw = (frame.as_pretty_frame().yaw as f32).to_radians();
                // info!("Yaw: {}", yaw);
            })
            .unwrap();

        let mut adc_line = [0u16; 32];
        let mut adc_ir = [0u16; 16];
        for i in 0..16 {
            if i & 0b0001 != 0 {
                line_s0.set_high();
            } else {
                line_s0.set_low();
            }
            if i & 0b0010 != 0 {
                line_s1.set_high();
            } else {
                line_s1.set_low();
            }
            if i & 0b0100 != 0 {
                line_s2.set_high();
            } else {
                line_s2.set_low();
            }
            if i & 0b1000 != 0 {
                line_s3.set_high();
            } else {
                line_s3.set_low();
            }

            if i & 0b0001 != 0 {
                ir_s0.set_high();
            } else {
                ir_s0.set_low();
            }
            if i & 0b0010 != 0 {
                ir_s1.set_high();
            } else {
                ir_s1.set_low();
            }
            if i & 0b0100 != 0 {
                ir_s2.set_high();
            } else {
                ir_s2.set_low();
            }
            if i & 0b1000 != 0 {
                ir_s3.set_high();
            } else {
                ir_s3.set_low();
            }

            adc_line[i] = adc1.read(&mut p.PC0);
            adc_line[i + 16] = adc1.read(&mut p.PC1);
            adc_ir[i] = adc1.read(&mut p.PC2);
        }

        let adc_line = adc_line
            .iter()
            .map(|x| (*x as f32) / 4096.0)
            .collect::<Vec<_>>();

        let (line_x, line_y) = calculate_adc_vec(&adc_line, &adc_line_sin, &adc_line_cos, 1.0);

        adc_ir.iter_mut().for_each(|x| *x = 4096 - *x);
        let adc_ir = adc_ir
            .iter()
            .map(|x| (*x as f32) / 4096.0)
            .collect::<Vec<_>>();

        let (ir_x, ir_y) = calculate_adc_vec(&adc_ir, &adc_ir_sin, &adc_ir_cos, 1.0);

        // info!("Line X: {}, Line Y: {}", line_x, line_y);
        // info!("IR X: {}, IR Y: {}", ir_x, ir_y);

        let msgpack = G_MSG_PACK_RX.lock().await.get_mut().clone();
        angle_pid.setpoint(msgpack.vel.angular_z);

        let angle_pid_output = angle_pid.next_control_output(yaw);

        let motor1 = wheel_calc1.calculate(
            msgpack.vel.linear_x,
            msgpack.vel.linear_y,
            angle_pid_output.output,
        );
        let motor2 = wheel_calc2.calculate(
            msgpack.vel.linear_x,
            msgpack.vel.linear_y,
            angle_pid_output.output,
        );
        let motor3 = wheel_calc3.calculate(
            msgpack.vel.linear_x,
            msgpack.vel.linear_y,
            angle_pid_output.output,
        );
        let motor4 = wheel_calc4.calculate(
            msgpack.vel.linear_x,
            msgpack.vel.linear_y,
            angle_pid_output.output,
        );

        motor_speed = MotorSpeed {
            motor1,
            motor2,
            motor3,
            motor4,
        };

        if gpio_ui_toggle.is_high() {
            motor_speed = MotorSpeed {
                motor1: 0.0,
                motor2: 0.0,
                motor3: 0.0,
                motor4: 0.0,
            };
        }

        let motor_speed_data = MdData { motor_speed };

        let mut buffer = [0u8; 64];
        let encoded_size = corncobs::encode_buf(unsafe { &motor_speed_data.buffer }, &mut buffer);

        match uart_md.write(&buffer[..encoded_size]).await {
            Ok(_) => {}
            Err(e) => {
                info!("UART write error: {:?}", e);
            }
        };
    }
}

#[embassy_executor::task]
async fn uart_task(
    mut uart_jetson: Uart<
        'static,
        peripherals::USART3,
        peripherals::DMA1_CH3,
        peripherals::DMA1_CH1,
    >,
) {
    const RX_DATA_SIZE: usize = 57;
    let msgpack_default = corepack::to_bytes(nv1_msg::HubMsgPackRx::default()).unwrap();
    let msgpack_data_len: usize = msgpack_default.len();

    loop {
        let mut buf = [0u8; RX_DATA_SIZE];
        match uart_jetson.read(&mut buf).await {
            Ok(_) => {
                let mut cobs_decoded_buf = [0; 64];
                match corncobs::decode_buf(&buf, &mut cobs_decoded_buf) {
                    Ok(size) => {
                        if size != msgpack_data_len {
                            info!("Invalid data size: {}", size);
                            continue;
                        }

                        let msgpack = corepack::from_bytes::<nv1_msg::HubMsgPackRx>(
                            &cobs_decoded_buf[0..msgpack_data_len],
                        )
                        .unwrap();

                        info!("Linear X: {}", msgpack.vel.linear_x);
                        info!("Linear Y: {}", msgpack.vel.linear_y);
                        info!("Angular Z: {}", msgpack.vel.angular_z);

                        G_MSG_PACK_RX.lock().await.get_mut().vel = msgpack.vel
                    }
                    Err(err) => {
                        info!("Failed to decode data");
                        info!("Data: {:?}", buf);
                        match err {
                            corncobs::CobsError::Truncated => info!("Truncated"),
                            corncobs::CobsError::Corrupt => info!("Corrupt"),
                        }
                    }
                }
            }
            Err(err) => {
                if err == usart::Error::Overrun {
                    info!("Overrun");
                    continue;
                } else {
                    info!("Failed to read data");
                    info!("Error: {:?}", err);
                    continue;
                }
            }
        };
    }
}
