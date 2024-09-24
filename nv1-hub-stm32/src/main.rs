// TODO ui MutexからRc moveに置き換える

#![no_std]
#![no_main]

mod fmt;
mod omni;

extern crate alloc;

use core::f32::consts::PI;
use core::{borrow::Borrow, cell::RefCell};

use alloc::rc::Rc;
use alloc::vec::Vec;
use alloc::{boxed::Box, vec};
use defmt::error;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration, Timer};
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
use embedded_graphics::prelude::{Point, Size};
use libm::{cosf, powf, sinf, sqrtf};
use num_traits::{AsPrimitive, Num};
use nv1_hub_ui::elements::{Element, Slider, SliderOption, Text, TextOption, Value, ValueOption};
use nv1_hub_ui::menu::{Menu, MenuOption};
use nv1_hub_ui::{
    elements::{Button, ButtonOption},
    menu::{ListMenu, ListMenuOption},
    Event, HubUI,
};
use nv1_hub_ui::{EventKey, HubUIOption};

use nv1_msg::hub::HubMsgPackTx;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
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

static BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();

static G_UART: Mutex<
    ThreadModeRawMutex,
    Option<Uart<'static, peripherals::USART3, peripherals::DMA1_CH3, peripherals::DMA1_CH1>>,
> = Mutex::new(None);

static G_MSG_RX: Mutex<ThreadModeRawMutex, nv1_msg::hub::HubMsgPackRx> =
    Mutex::new(nv1_msg::hub::HubMsgPackRx {
        vel: nv1_msg::hub::Velocity {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        kick: false,
    });

static G_MSG_TX: Mutex<ThreadModeRawMutex, RefCell<nv1_msg::hub::HubMsgPackTx>> =
    Mutex::new(RefCell::new(nv1_msg::hub::HubMsgPackTx {
        pause: false,
        shutdown: false,
        reboot: false,
        vel: nv1_msg::hub::Velocity {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        ir: nv1_msg::hub::Ir {
            x: 0.0,
            y: 0.0,
            strength: 0.0,
        },
        line: nv1_msg::hub::Line {
            x: 0.0,
            y: 0.0,
            strength: 0.0,
        },
        have_ball: false,
    }));

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

fn calculate_adc_vec<T>(adc: &[T], adc_sin: &[T], adc_cos: &[T], _mul: T) -> (f32, f32, T)
where
    T: Num + Copy + 'static + AsPrimitive<f32> + PartialOrd,
    f32: AsPrimitive<T>,
{
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;
    let mut max_adc: T = T::zero();

    for i in 0..adc.len() {
        if adc[i] > max_adc {
            max_adc = adc[i];
        }
        sum_x = sum_x + (adc_cos[i] * adc[i]).as_();
        sum_y = sum_y + (adc_sin[i] * adc[i]).as_();
    }

    let norm = sqrtf(powf(sum_x / adc.len() as f32, 2.0) + powf(sum_y / adc.len() as f32, 2.0));

    (
        sum_x / adc.len() as f32 / norm,
        sum_y / adc.len() as f32 / norm,
        max_adc,
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
    let uart_jetson = Uart::new(
        p.USART3,
        p.PC5,
        p.PB10,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH1,
        uart3_config,
    )
    .unwrap();

    G_UART.lock().await.replace(uart_jetson);

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

    let mut gpio_reset = Output::new(p.PA0, Level::High, embassy_stm32::gpio::Speed::Low);
    gpio_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    gpio_reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

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

    let mut adc_ir_sin = [0.0f32; 16];
    let mut adc_ir_cos = [0.0f32; 16];

    generate_adc_vec(
        &mut adc_ir_sin,
        &mut adc_ir_cos,
        (90 as f32).to_radians(),
        -((360.0 / 16.0) as f32).to_radians(),
        1.0,
    );

    let f = Rc::new(RefCell::new(Flash::new_blocking(p.FLASH)));

    let settings = Rc::new(RefCell::new(
        flash_read(&mut f.clone().borrow_mut()).unwrap_or(Settings {
            line_strength: 0.12,
        }),
    ));

    if settings.borrow_mut().line_strength.is_nan() {
        settings.borrow_mut().line_strength = 0.12;

        flash_write(&mut f.clone().borrow_mut(), &settings.borrow_mut()).unwrap();
    }

    info!("Line strength: {}", settings.borrow_mut().line_strength);

    let gpio_ui_toggle = Input::new(p.PC12, Pull::None);
    let gpio_ui_up = Input::new(p.PC13, Pull::None);
    let gpio_ui_down = Input::new(p.PC14, Pull::None);
    let gpio_ui_enter = Input::new(p.PC15, Pull::None);

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
    let mut ssd1306_init_success = false;
    match ssd1306.init() {
        Ok(_) => {
            ssd1306_init_success = true;
        }
        Err(_) => {
            error!("Can't initialize ssd1306");
        }
    };

    let ui_option = HubUIOption {
        menu_option: MenuOption {
            position: Point::new(2 + 64, 2),
            size: Size::new(64 - 4, 64 - 4),
        },
    };

    let ui_text = Text::new(
        "Interface",
        TextOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let mut shutdown = Rc::new(RefCell::new(false));

    let shutdown_clone = shutdown.clone();
    let ui_shutdown = Button::new(
        "Shutdown",
        move |pressed| {
            shutdown_clone.replace(pressed);
        },
        ButtonOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let mut reboot = Rc::new(RefCell::new(false));

    let reboot_clone = reboot.clone();
    let ui_reboot = Button::new(
        "Reboot",
        move |pressed| {
            reboot_clone.replace(pressed);
        },
        ButtonOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let mut line_value = Rc::new(RefCell::new(0.0));

    let line_value_clone = line_value.clone();
    let mut ui_line_value = Value::new(
        "L",
        0.0,
        |value| {
            *value = *line_value_clone.borrow_mut();
        },
        ValueOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_line_strength = Slider::new(
        settings.borrow_mut().line_strength,
        0.0,
        1.0,
        0.01,
        move |value| {
            settings_clone.borrow_mut().line_strength = value;
            flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
        },
        SliderOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let settings_clone = settings.clone();
    let ui_settings_reset = Button::new(
        "S Reset",
        move |pressed| {
            if pressed {
                settings_clone.borrow_mut().line_strength = 0.12;
                flash_write(&mut f.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
                settings_clone.replace(flash_read(&mut f.borrow_mut()).unwrap());
            }
        },
        ButtonOption {
            font: embedded_graphics::mono_font::ascii::FONT_6X10,
        },
    );

    let elements: Vec<
        Box<
            dyn Element<
                Ssd1306<
                    I2CInterface<I2c<peripherals::I2C3>>,
                    DisplaySize128x64,
                    BufferedGraphicsMode<DisplaySize128x64>,
                >,
            >,
        >,
    > = vec![
        Box::new(ui_text),
        Box::new(ui_shutdown),
        Box::new(ui_reboot),
        Box::new(ui_line_value),
        Box::new(ui_line_strength),
        Box::new(ui_settings_reset),
    ];
    let menu: Vec<
        Box<
            dyn Menu<
                Ssd1306<
                    I2CInterface<I2c<peripherals::I2C3>>,
                    DisplaySize128x64,
                    BufferedGraphicsMode<DisplaySize128x64>,
                >,
            >,
        >,
    > = vec![Box::new(ListMenu::new(
        elements,
        ListMenuOption {
            vertical_num: 4,
            element_margin: 1,
            cursor_line_len: 4,
        },
    ))];
    let mut ui = HubUI::new(&mut ssd1306, menu, ui_option);
    let display = ui.update(&Event::None);
    if ssd1306_init_success {
        display.flush().unwrap();
    }

    let (mut proc, mut parser) = match bno08x_rvc::create(BB.borrow()) {
        Ok((proc, pars)) => (proc, pars),
        Err(_e) => {
            error!("Can't create bno08x-rvc");
            loop {}
        }
    };

    let mut yaw = 0.0;

    let mut rotation_pid: pid::Pid<f32> = pid::Pid::new(0.0, 100.0);
    rotation_pid.p(7.0, 100.0);

    const WHEEL_R: f32 = 25.0 / 1000.0;
    const THREAD: f32 = 108.0 / 1000.0;
    let wheel_calc1 = omni::OmniWheel::new(45.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc2 = omni::OmniWheel::new(315.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc3 = omni::OmniWheel::new(225.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc4 = omni::OmniWheel::new(135.0_f32.to_radians(), WHEEL_R, THREAD);

    spawner.spawn(uart_jetson_rx_task()).unwrap();
    spawner.spawn(uart_jetson_tx_task()).unwrap();

    let mut loop_count = 0;

    info!("nv1-hub initialized");

    let shutdown = shutdown.clone();
    let reboot = reboot.clone();

    loop {
        let mut buf = [0u8; 19];
        uart_bno.read(&mut buf).await.unwrap();
        proc.process_slice(&buf).unwrap();
        parser
            .worker(|frame| {
                yaw = -((frame.as_pretty_frame().yaw as f32).to_radians());
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

        let (line_x, line_y, line_strength) =
            calculate_adc_vec(&adc_line, &adc_line_sin, &adc_line_cos, 1.0);

        adc_ir.iter_mut().for_each(|x| *x = 4096 - *x);
        let adc_ir = adc_ir
            .iter()
            .map(|x| (*x as f32) / 4096.0)
            .collect::<Vec<_>>();

        let adc_ir_over_count = adc_ir.iter().filter(|x| **x > 0.06).count();

        let (ir_x, ir_y, _ir_strength) = calculate_adc_vec(&adc_ir, &adc_ir_sin, &adc_ir_cos, 1.0);

        // info!(
        //     "angle: {}, IR X: {}, IR Y: {}",
        //     libm::atan2f(ir_x, ir_y),
        //     ir_x,
        //     ir_y
        // );

        let adc_have_ball = adc1.read(&mut p.PC3);

        let mut additional_vel_x = 0.0;
        let mut additional_vel_y = 0.0;
        // info!("line_strength: {}", line_strength);

        info!("line_strength: {}", settings.borrow_mut().line_strength);

        // Line detect
        if line_strength > settings.borrow_mut().line_strength {
            // info!("[LINE] Line detected");
            additional_vel_x = -line_x * 4.0;
            additional_vel_y = -line_y * 4.0;
        }

        let msg = G_MSG_RX.lock().await.clone();

        let vel_x = msg.vel.x * 1.0 + additional_vel_x;
        let vel_y = msg.vel.y * 1.0 + additional_vel_y;
        // let rotation_target = -msg.vel.angle; // reversed
        let rotation_target = 0.0;

        // info!("rotation_target: {}", rotation_target);

        rotation_pid.setpoint(rotation_target);

        let rotation_pid_result = rotation_pid.next_control_output(yaw);
        let rotation_vel = rotation_pid_result.output;

        let motor1 = wheel_calc1.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor2 = wheel_calc2.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor3 = wheel_calc3.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor4 = wheel_calc4.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);

        // info!(
        //     "Motor1: {}, Motor2: {}, Motor3: {}, Motor4: {}",
        //     motor1, motor2, motor3, motor4
        // );

        let mut md_msg = nv1_msg::md::HubMsgPackRx {
            enable: true,
            m1: motor1,
            m2: motor2,
            m3: motor3,
            m4: motor4,
        };

        if gpio_ui_toggle.is_high() {
            md_msg = nv1_msg::md::HubMsgPackRx {
                enable: false,
                m1: 0.0,
                m2: 0.0,
                m3: 0.0,
                m4: 0.0,
            };
        }

        let md_data = postcard::to_vec_cobs::<nv1_msg::md::HubMsgPackRx, 64>(&md_msg).unwrap();
        match uart_md.write(&md_data).await {
            Ok(_) => {}
            Err(err) => {
                error!("[UART MD] write error: {:?}", err);
            }
        };

        let mut ir_distance = (8 - adc_ir_over_count) as f32 * 0.15;
        if ir_distance < 0.0 || ir_distance > 2.0 {
            ir_distance = 0.0;
        }
        // info!("IR distance: {}", ir_distance);

        if loop_count % 10 == 0 {
            let event = if gpio_ui_up.is_high() {
                Event::KeyDown(EventKey::Up)
            } else if gpio_ui_down.is_high() {
                Event::KeyDown(EventKey::Down)
            } else if gpio_ui_enter.is_high() {
                Event::KeyDown(EventKey::Enter)
            } else {
                Event::None
            };
            let display = ui.update(&event);
            if ssd1306_init_success {
                display.flush().unwrap();
            }
        }

        let msg_tx = HubMsgPackTx {
            pause: gpio_ui_toggle.is_high(),
            shutdown: *shutdown.borrow_mut(),
            reboot: *reboot.borrow_mut(),
            vel: nv1_msg::hub::Velocity {
                x: msg.vel.x,
                y: msg.vel.y,
                angle: yaw,
            },
            ir: nv1_msg::hub::Ir {
                x: ir_x,
                y: ir_y,
                strength: ir_distance,
            },
            line: nv1_msg::hub::Line {
                x: line_x,
                y: line_y,
                strength: 0.0,
            },
            have_ball: adc_have_ball < 2048,
        };

        G_MSG_TX.lock().await.replace(msg_tx);

        loop_count += 1;
    }
}

#[embassy_executor::task]
async fn uart_jetson_rx_task() {
    const RX_DATA_SIZE: usize = 15;

    let mut timeout_count = 0;

    loop {
        let mut msg_with_cobs = [0u8; RX_DATA_SIZE];
        let timeout_res = with_timeout(
            Duration::from_millis(5),
            G_UART
                .lock()
                .await
                .as_mut()
                .unwrap()
                .read(&mut msg_with_cobs),
        )
        .await;
        match timeout_res {
            Ok(rx) => match rx {
                Ok(_) => {
                    // info!("[UART Jetson] received data: {:?}", msg_with_cobs);
                    match postcard::from_bytes_cobs::<nv1_msg::hub::HubMsgPackRx>(
                        &mut msg_with_cobs,
                    ) {
                        Ok(msg) => {
                            // info!("Linear X: {}", msg.vel.x);
                            // info!("Linear Y: {}", msg.vel.y);
                            // info!("Angular Z: {}", msg.vel.angle);

                            G_MSG_RX.lock().await.vel = msg.vel;
                        }
                        Err(_) => {
                            error!("[UART Jetson] postcard decode error");
                        }
                    };
                    timeout_count = 0;
                }
                Err(err) => {
                    error!("[UART Jetson] read error: {:?}", err);
                }
            },
            Err(_) => {
                timeout_count += 1;

                if timeout_count > 10 {
                    error!("[UART Jetson] timeout");
                    G_MSG_RX.lock().await.vel = nv1_msg::hub::Velocity {
                        x: 0.0,
                        y: 0.0,
                        angle: 0.0,
                    };
                    timeout_count = 0;
                }
            }
        }
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn uart_jetson_tx_task() {
    loop {
        let msg = G_MSG_TX.lock().await.take();
        match postcard::to_vec_cobs::<nv1_msg::hub::HubMsgPackTx, 64>(&msg) {
            Ok(msg_with_cobs) => {
                match G_UART
                    .lock()
                    .await
                    .as_mut()
                    .unwrap()
                    .write(&msg_with_cobs)
                    .await
                {
                    Ok(_) => {
                        // info!("[UART Jetson] sent data, len: {}", msg_with_cobs.len());
                    }
                    Err(e) => {
                        error!("[UART Jetson] write error: {:?}", e);
                    }
                };
            }
            Err(_) => {
                error!("[UART Jetson] postcard encode error");
            }
        }
        Timer::after_millis(10).await;
    }
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
struct Settings {
    pub line_strength: f32,
}

fn flash_read(f: &mut Flash<'_, Blocking>) -> Result<Settings, embassy_stm32::flash::Error> {
    let mut buf = [0u8; 32];
    f.blocking_read(128 * 1024, &mut buf)?;

    Ok(postcard::from_bytes(&buf).unwrap())
}

fn flash_write(
    f: &mut Flash<'_, Blocking>,
    settings: &Settings,
) -> Result<(), embassy_stm32::flash::Error> {
    let mut buf = [0u8; 32];
    postcard::to_slice(settings, &mut buf).unwrap();

    f.blocking_erase(128 * 1024, 128 * 1024 + 128 * 1024)?;
    f.blocking_write(128 * 1024, &buf)?;

    Ok(())
}
