// TODO ui MutexからRc moveに置き換える

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod fmt;
mod neo_pixel;
mod omni;

extern crate alloc;

use embedded_alloc::LlffHeap as Heap;
use nv1_msg::hub::HSV;
use serde::{Deserialize, Serialize};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::f32::consts::PI;
use core::{borrow::Borrow, cell::RefCell};

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;

use bbqueue::BBBuffer;
use defmt::error;
use embassy_executor::Spawner;
use embassy_futures::select::select3;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::OutputType;
use embassy_stm32::mode;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    gpio::{Level, Output, Pull},
    i2c::{self, I2c},
    peripherals,
    time::Hertz,
    usart::{self, Uart},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_graphics::prelude::{Point, Size};
use fmt::info;
use neo_pixel::NeoPixelPwm;
use num_traits::{AsPrimitive, Num};
use nv1_hub_ui::elements;
use nv1_hub_ui::elements::Element;
use nv1_hub_ui::elements::{Slider, Text, Value};
use nv1_hub_ui::menu::Menu;
use nv1_hub_ui::{
    elements::Button,
    menu::{ListMenu, ListMenuOption},
    Event, HubUI,
};
use nv1_hub_ui::{menus, EventKey, HubUIOption};
use rgb::RGB8;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::{mode::DisplayConfig, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use static_cell::StaticCell;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

// const IR_ANGLE_THRESHOLD: f32 = 90_f32.to_radians() / 2.0;
// const IR_COUNT_THRESHOLD: f32 = 0.02;
const LINE_OVER_CENTER_THRESHOLD: f32 = 120_f32.to_radians();
const LOOP_US: u64 = 3000;
const DEFAULT_SETTINGS: Settings = Settings {
    line_strength: 0.12,
    opp_goal_color: GoalColor::Blue,
    opencv_goal_blue: HSV {
        h_min: 100,
        h_max: 120,
        s_min: 100,
        s_max: 255,
        v_min: 100,
        v_max: 255,
    },
    opencv_goal_yellow: HSV {
        h_min: 0,
        h_max: 255,
        s_min: 0,
        s_max: 255,
        v_min: 0,
        v_max: 255,
    },
};

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

static G_BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();
static G_YAW: Mutex<ThreadModeRawMutex, RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_JETSON_RX: Mutex<ThreadModeRawMutex, nv1_msg::hub::ToHub> =
    Mutex::new(nv1_msg::hub::ToHub {
        vel: nv1_msg::hub::Movement {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        kick: false,
        goal_opp: None,
        goal_own: None,
    });
static G_JETSON_TX: Mutex<ThreadModeRawMutex, RefCell<nv1_msg::hub::ToJetson>> =
    Mutex::new(RefCell::new(nv1_msg::hub::ToJetson {
        sys: nv1_msg::hub::System {
            pause: false,
            shutdown: false,
            reboot: false,
        },
        vel: nv1_msg::hub::Movement {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        sensor: nv1_msg::hub::Sensor {
            ir: nv1_msg::hub::Ir {
                x: 0.0,
                y: 0.0,
                strength: 0.0,
            },
            on_line: false,
            have_ball: false,
        },
        config: nv1_msg::hub::JetsonConfig::None,
    }));
static G_NEO_PIXEL_DATA: Mutex<ThreadModeRawMutex, NeoPixelData> = Mutex::new(NeoPixelData {
    jetson_connecting: false,
    pause: false,
    ball_dir: 0.0,
});

fn generate_adc_vec<T>(sin: &mut [T], cos: &mut [T], offset: f32, one_angle: f32, mul: f32)
where
    f32: AsPrimitive<T>,
    T: Num + Copy + 'static,
{
    for i in 0..sin.len() {
        sin[i] = (libm::sinf(i as f32 * one_angle + offset) * mul).as_();
        cos[i] = (libm::cosf(i as f32 * one_angle + offset) * mul).as_();
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

    let norm = libm::sqrtf(
        libm::powf(sum_x / adc.len() as f32, 2.0) + libm::powf(sum_y / adc.len() as f32, 2.0),
    );

    (
        sum_x / adc.len() as f32 / norm,
        sum_y / adc.len() as f32 / norm,
        max_adc,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
enum GoalColor {
    #[default]
    Blue,
    Yellow,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // initialize static heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 2048;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut config = embassy_stm32::Config::default();

    {
        use embassy_stm32::rcc;

        config.rcc.hse = Some(rcc::Hse {
            freq: Hertz(20_000_000),
            mode: rcc::HseMode::Oscillator,
        });

        config.rcc.pll_src = rcc::PllSource::HSE;
        config.rcc.pll = Some(rcc::Pll {
            prediv: rcc::PllPreDiv::DIV16,
            mul: rcc::PllMul::MUL288,
            divp: Some(rcc::PllPDiv::DIV2),
            divq: None,
            divr: Some(rcc::PllRDiv::DIV2),
        });

        config.rcc.sys = rcc::Sysclk::PLL1_P;

        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }

    // initialize peripherals
    let mut p = embassy_stm32::init(config);

    info!("Hello, world!");

    // initialize UARTs
    let mut uart_jetson_config = usart::Config::default();
    uart_jetson_config.baudrate = 115200;
    let uart_jetson = Uart::new(
        p.USART3,
        p.PC5,
        p.PB10,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH1,
        uart_jetson_config,
    )
    .unwrap();

    let mut uart_md_config = usart::Config::default();
    uart_md_config.baudrate = 115200;
    let mut uart_md = Uart::new(
        p.UART4,
        p.PC11,
        p.PC10,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH2,
        uart_md_config,
    )
    .unwrap();

    let mut uart_bno_config = usart::Config::default();
    uart_bno_config.baudrate = bno08x_rvc::BNO08X_UART_RVC_BAUD_RATE;
    let mut uart_bno = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        Irqs,
        p.DMA2_CH6,
        p.DMA2_CH1,
        uart_bno_config,
    )
    .unwrap();
    // reset bno08x
    let mut gpio_reset = Output::new(p.PA0, Level::High, embassy_stm32::gpio::Speed::Low);
    gpio_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    gpio_reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // initialize ADC
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES3);

    let mut line_s0 = Output::new(p.PB12, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut line_s1 = Output::new(p.PB13, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut line_s2 = Output::new(p.PB14, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut line_s3 = Output::new(p.PB15, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);

    let mut ir_s0 = Output::new(p.PB0, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut ir_s1 = Output::new(p.PB1, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut ir_s2 = Output::new(p.PB4, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let mut ir_s3 = Output::new(p.PB5, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);

    let mut adc_line_sin = [0.0_f32; 32];
    let mut adc_line_cos = [0.0_f32; 32];
    generate_adc_vec(
        &mut adc_line_sin,
        &mut adc_line_cos,
        90.0_f32.to_radians(),
        -(360.0_f32 / 32.0_f32).to_radians(),
        1.0,
    );
    info!("adc_line_sin: {:?}", adc_line_sin);
    info!("adc_line_cos: {:?}", adc_line_cos);

    let mut adc_ir_sin = [0.0_f32; 16];
    let mut adc_ir_cos = [0.0_f32; 16];
    generate_adc_vec(
        &mut adc_ir_sin,
        &mut adc_ir_cos,
        90_f32.to_radians(),
        -(360.0_f32 / 16.0_f32).to_radians(),
        1.0,
    );

    // initialize flash
    let f = Rc::new(RefCell::new(Flash::new_blocking(p.FLASH)));
    let settings = Rc::new(RefCell::new(
        flash_read(&mut f.clone().borrow_mut()).unwrap_or(DEFAULT_SETTINGS),
    ));
    if settings.borrow_mut().line_strength.is_nan() {
        settings.borrow_mut().line_strength = 0.12;

        flash_write(&mut f.clone().borrow_mut(), &settings.borrow_mut()).unwrap();
    }

    info!("line strength: {}", settings.borrow_mut().line_strength);

    // UI
    let gpio_ui_toggle = ExtiInput::new(p.PC12, p.EXTI12, Pull::None);
    let gpio_ui_up = ExtiInput::new(p.PC13, p.EXTI13, Pull::None);
    let gpio_ui_down = ExtiInput::new(p.PC14, p.EXTI14, Pull::None);
    let gpio_ui_enter = ExtiInput::new(p.PC15, p.EXTI15, Pull::None);

    let mut config = i2c::Config::default();
    config.timeout = Duration::from_millis(100);
    let ssd1306_i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, Hertz::khz(400), config);

    let ssd1306_interface = I2CDisplayInterface::new(ssd1306_i2c);
    let ssd1306 = Ssd1306::new(
        ssd1306_interface,
        DisplaySize128x64,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    static SSD1306: StaticCell<
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    > = StaticCell::new();

    let ssd1306: &'static mut Ssd1306<
        I2CInterface<I2c<mode::Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    > = SSD1306.init(ssd1306);

    let mut ssd1306_init_success = false;
    match ssd1306.init() {
        Ok(_) => {
            ssd1306_init_success = true;
        }
        Err(_) => {
            error!("Can't initialize ssd1306");
        }
    };

    // UI view
    let ui_text_interface = Text::new("INTERFACE", embedded_graphics::mono_font::ascii::FONT_6X10);

    let shutdown = Rc::new(RefCell::new(false));
    let shutdown_clone = shutdown.clone();
    let ui_button_shutdown = Button::new(
        "Shutdown",
        move |pressed| {
            shutdown_clone.replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let reboot = Rc::new(RefCell::new(false));
    let reboot_clone = reboot.clone();
    let ui_button_reboot = Button::new(
        "Reboot",
        move |pressed| {
            reboot_clone.replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let ui_value_coat = Value::new(
        "Opp",
        "",
        move |value| {
            *value = match settings_clone.borrow_mut().opp_goal_color {
                GoalColor::Blue => "B",
                GoalColor::Yellow => "Y",
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_button_coat_change = Button::new(
        "Switch Coat",
        move |pressed| {
            if pressed {
                let toggle_color = match settings_clone.as_ref().borrow().opp_goal_color {
                    GoalColor::Blue => GoalColor::Yellow,
                    GoalColor::Yellow => GoalColor::Blue,
                };
                settings_clone.borrow_mut().opp_goal_color = toggle_color;
                flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let line_value = Rc::new(RefCell::new(0.0));
    let line_value_clone = line_value.clone();
    let ui_value_line = Value::new(
        "L",
        0.0,
        move |value| {
            *value = *line_value_clone.borrow_mut();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_slider_line_strength = Slider::new(
        settings.borrow_mut().line_strength,
        0.0,
        1.0,
        0.01,
        move |value| {
            settings_clone.borrow_mut().line_strength = value;
            flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_button_settings_reset = Button::new(
        "Reset",
        move |pressed| {
            if pressed {
                flash_write(&mut f_clone.borrow_mut(), &DEFAULT_SETTINGS).unwrap();
                settings_clone.replace(flash_read(&mut f_clone.borrow_mut()).unwrap());
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let elements = elements![
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        ui_text_interface,
        ui_button_shutdown,
        ui_button_reboot,
        ui_value_coat,
        ui_button_coat_change,
        ui_value_line,
        ui_slider_line_strength,
        ui_button_settings_reset
    ];
    let menu = menus![
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        ListMenu::new(
            elements,
            ListMenuOption {
                position: Point::new(64, 0),
                size: Size::new(64, 64),
                vertical_num: 4,
                element_margin: 1,
                cursor_line_len: 4,
            },
        )
    ];

    let ui_option = HubUIOption {};

    let mut ui = HubUI::new(ssd1306, menu, ui_option);
    let display = ui.update(&Event::None);
    if ssd1306_init_success {
        ssd1306_init_success = display.flush().is_ok();
    }

    let shutdown = shutdown.clone();
    let reboot = reboot.clone();

    let neo_pixel_pwm_hz = Hertz::khz(500);
    let neo_pixel_pwm = SimplePwm::new(
        p.TIM4,
        Some(PwmPin::new_ch1(p.PB6, OutputType::PushPull)),
        None,
        None,
        None,
        neo_pixel_pwm_hz,
        CountingMode::EdgeAlignedUp,
    );
    let neo_pixel = NeoPixelPwm::new(neo_pixel_pwm, neo_pixel_pwm_hz);
    static NEO_PIXEL_DMA: StaticCell<peripherals::DMA1_CH0> = StaticCell::new();
    let neo_pixel_dma: &'static mut peripherals::DMA1_CH0 = NEO_PIXEL_DMA.init(p.DMA1_CH0);

    let mut rotation_pid: pid::Pid<f32> = pid::Pid::new(0.0, 100.0);
    rotation_pid.p(8.0, 100.0);

    const WHEEL_R: f32 = 25.0 / 1000.0;
    const THREAD: f32 = 108.0 / 1000.0;
    let wheel_calc1 = omni::OmniWheel::new(45.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc2 = omni::OmniWheel::new(315.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc3 = omni::OmniWheel::new(225.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc4 = omni::OmniWheel::new(135.0_f32.to_radians(), WHEEL_R, THREAD);

    enum AdcState {
        OnGround,
        OnLine(f32, f32, f32),
        OutOfLineOverCenter(f32, f32, f32, u32),
    }
    let mut prev_adc_state = AdcState::OnGround;

    spawner.must_spawn(bno08x_task(uart_bno));
    spawner.must_spawn(uart_jetson_task(uart_jetson));
    if ssd1306_init_success {
        spawner.must_spawn(ui_task(ui, gpio_ui_up, gpio_ui_down, gpio_ui_enter));
    }
    spawner.must_spawn(neo_pixel_task(neo_pixel, neo_pixel_dma));

    info!("[nv1-hub] initialized");

    let mut prev_time = Instant::now();
    loop {
        let yaw = G_YAW.lock().await.clone().take();

        // 600us
        let mut adc_line = [0u16; 32];
        let mut adc_ir = [0u16; 16];
        let adc_have_ball = adc.blocking_read(&mut p.PC3);
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

            adc_line[i] = adc.blocking_read(&mut p.PC0);
            adc_line[i + 16] = adc.blocking_read(&mut p.PC1);
            adc_ir[i] = adc.blocking_read(&mut p.PC2);
        }

        // 350us
        let adc_line = adc_line
            .iter()
            .map(|x| (*x as f32) / 4096.0)
            .collect::<Vec<_>>();

        // 1000us
        let (line_vel_x, line_vel_y, line_strength) =
            calculate_adc_vec(&adc_line, &adc_line_sin, &adc_line_cos, 1.0);

        // 200us?
        let line_vel: Option<(f32, f32)> = match prev_adc_state {
            AdcState::OnGround => {
                if line_strength > settings.borrow_mut().line_strength {
                    // prev: on Ground, now: on Line
                    info!("[LINE] Line detected");

                    let now_angle = libm::atan2f(line_vel_y, line_vel_x);
                    let now_angle = if now_angle < 0.0 {
                        2.0 * PI + now_angle
                    } else {
                        now_angle
                    };

                    prev_adc_state = AdcState::OnLine(now_angle, line_vel_x, line_vel_y);
                    Some((-line_vel_x, -line_vel_y))
                } else {
                    // prev: on Ground, now: on Ground
                    prev_adc_state = AdcState::OnGround;
                    None
                }
            }
            AdcState::OnLine(old_angle, old_line_x, old_line_y) => {
                if line_strength < settings.borrow_mut().line_strength {
                    // prev: on Line, now: on Ground
                    prev_adc_state = AdcState::OnGround;
                    None
                } else {
                    // prev: on Line, now: on Line
                    let now_angle = libm::atan2f(line_vel_y, line_vel_x); // -3.14 ~ 3.14
                    let now_angle = if now_angle < 0.0 {
                        2.0 * PI + now_angle
                    } else {
                        now_angle
                    };

                    if libm::fabsf(old_angle - now_angle) > LINE_OVER_CENTER_THRESHOLD {
                        // prev: on Line, now: out of Center

                        prev_adc_state =
                            AdcState::OutOfLineOverCenter(old_angle, old_line_x, old_line_y, 0);
                        info!(
                            "[LINE] Out of line new_angle: {}, prev_angle: {}",
                            now_angle, old_angle
                        );
                        Some((-line_vel_x, -line_vel_y))
                    } else {
                        // prev: on Line, now: on Line
                        prev_adc_state = AdcState::OnLine(old_angle, old_line_x, old_line_y);

                        Some((-line_vel_x, -line_vel_y))
                    }
                }
            }
            AdcState::OutOfLineOverCenter(old_angle, old_line_x, old_line_y, counter) => {
                let now_angle = libm::atan2f(line_vel_y, line_vel_x);
                let now_angle = if now_angle < 0.0 {
                    2.0 * PI + now_angle
                } else {
                    now_angle
                };

                if line_strength > settings.borrow_mut().line_strength
                    && libm::fabsf(old_angle - now_angle) < LINE_OVER_CENTER_THRESHOLD
                {
                    prev_adc_state = AdcState::OnLine(now_angle, line_vel_x, line_vel_y);
                    Some((-line_vel_x, -line_vel_y))
                } else if counter > 100 {
                    // emergency!!

                    info!("[LINE] Emergency!!");
                    prev_adc_state = AdcState::OnGround;
                    None
                } else {
                    prev_adc_state = AdcState::OutOfLineOverCenter(
                        old_angle,
                        old_line_x,
                        old_line_y,
                        counter + 1,
                    );
                    Some((-old_line_x, -old_line_y))
                }
            }
        };

        // 1000us
        let adc_line_max = adc_line.into_iter().reduce(f32::max).unwrap_or(0.);
        line_value.replace(adc_line_max);
        adc_ir.iter_mut().for_each(|x| *x = 4096 - *x);
        let adc_ir = adc_ir
            .iter()
            .map(|x| (*x as f32) / 4096.0)
            .collect::<Vec<_>>();
        let (ir_x, ir_y, _ir_strength) = calculate_adc_vec(&adc_ir, &adc_ir_sin, &adc_ir_cos, 1.0);
        // let adc_ir_over_count = adc_ir.iter().filter(|x| **x > IR_COUNT_THRESHOLD).count();
        // info!("IR over count: {}", adc_ir_over_count);
        let ir_angle = libm::atan2f(ir_y, ir_x);
        // info!("IR angle: {}", ir_angle);
        // let ir_vel = if adc_ir_over_count > 10
        //     && ir_angle > PI / 2.0 - IR_ANGLE_THRESHOLD
        //     && ir_angle < PI / 2.0 + IR_ANGLE_THRESHOLD
        // {
        //     Some((ir_x * 0.8, 0.4))
        // } else {
        //     None
        // };
        // info!("line_strength: {}", line_strength);
        // info!("line_strength: {}", settings.borrow_mut().line_strength);

        let received_msg = G_JETSON_RX.lock().await.clone();
        let (vel_x, vel_y) = if let Some((line_vel_x, line_vel_y)) = line_vel {
            info!("[LINE] Line detected");
            (line_vel_x * 2.0, line_vel_y * 2.0)
        } else {
            (received_msg.vel.x * 1.5, received_msg.vel.y * 1.5)
        };
        // info!("Vel X: {}, Vel Y: {}", vel_x, vel_y);

        // 1000us~1300us
        let rotation_target = 0.0;
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

        let pause = gpio_ui_toggle.is_high();
        let md_msg = if pause {
            nv1_msg::md::ToMD {
                enable: false,
                m1: 0.0,
                m2: 0.0,
                m3: 0.0,
                m4: 0.0,
            }
        } else {
            nv1_msg::md::ToMD {
                enable: true,
                m1: motor1,
                m2: motor2,
                m3: motor3,
                m4: motor4,
            }
        };

        let md_data = postcard::to_vec_cobs::<nv1_msg::md::ToMD, 64>(&md_msg).unwrap();
        match uart_md.write(&md_data).await {
            Ok(_) => {}
            Err(err) => {
                error!("[UART MD] write error: {:?}", err);
            }
        };

        let opp_color = match settings.as_ref().borrow().opp_goal_color {
            GoalColor::Blue => settings.as_ref().borrow().opencv_goal_blue,
            GoalColor::Yellow => settings.as_ref().borrow().opencv_goal_yellow,
        };

        let own_color = match settings.as_ref().borrow().opp_goal_color {
            GoalColor::Blue => settings.as_ref().borrow().opencv_goal_yellow,
            GoalColor::Yellow => settings.as_ref().borrow().opencv_goal_blue,
        };

        let settings = settings.borrow_mut();
        let msg_tx = nv1_msg::hub::ToJetson {
            sys: nv1_msg::hub::System {
                pause,
                shutdown: *shutdown.borrow_mut(),
                reboot: *reboot.borrow_mut(),
            },
            vel: nv1_msg::hub::Movement {
                x: received_msg.vel.x,
                y: received_msg.vel.y,
                angle: yaw,
            },
            sensor: nv1_msg::hub::Sensor {
                ir: nv1_msg::hub::Ir {
                    x: ir_x,
                    y: ir_y,
                    strength: 0.0,
                },

                on_line: line_strength > settings.line_strength,
                have_ball: adc_have_ball < 800,
            },
            config: nv1_msg::hub::JetsonConfig::OpenCV(nv1_msg::hub::OpenCVConfig {
                opp_color,
                own_color,
            }),
        };
        G_JETSON_TX.lock().await.replace(msg_tx);

        G_NEO_PIXEL_DATA.lock().await.ball_dir = ir_angle;
        G_NEO_PIXEL_DATA.lock().await.pause = pause;

        let now_time = Instant::now();
        let elapsed_time = now_time - prev_time;
        info!("elapsed time: {}", elapsed_time.as_micros());
        if LOOP_US > elapsed_time.as_micros() {
            Timer::after_micros(LOOP_US - elapsed_time.as_micros()).await;
        }
        prev_time = now_time;
    }
}

#[embassy_executor::task]
async fn bno08x_task(mut uart: Uart<'static, mode::Async>) {
    let mut bno08x_buf = [0u8; 19];

    let (mut processor, mut parser) = match bno08x_rvc::create(G_BB.borrow()) {
        Ok((proc, pars)) => (proc, pars),
        Err(_e) => {
            error!("Can't create bno08x-rvc");
            loop {}
        }
    };

    loop {
        let _ = uart.read(&mut bno08x_buf).await;
        processor.process_slice(&bno08x_buf).unwrap();
        let _ = parser.worker(|frame| {
            let yaw = embassy_futures::block_on(G_YAW.lock());
            yaw.replace(-(frame.as_pretty_frame().yaw.to_radians()));
        });
    }
}

#[embassy_executor::task]
async fn uart_jetson_task(uart: Uart<'static, mode::Async>) {
    let (mut uart_tx, uart_rx) = uart.split();

    let mut dma_buf = [0u8; 128];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut dma_buf);

    uart_rx.start_uart();

    loop {
        let mut byte = [0u8; 1];
        let mut msg_with_cobs = [0u8; 64];
        let mut c = 0;
        loop {
            let timeout_res =
                with_timeout(Duration::from_millis(50), uart_rx.read(&mut byte)).await;
            match timeout_res {
                Ok(receive_res) => match receive_res {
                    Ok(_size) => {
                        msg_with_cobs[c] = byte[0];
                        c += 1;

                        if byte[0] == 0 {
                            break;
                        }
                    }
                    Err(err) => {
                        error!("[UART Jetson] read error: {:?}", err);
                        uart_rx.start_uart();
                    }
                },
                Err(_) => {
                    error!("[UART Jetson] timeout");

                    G_JETSON_RX.lock().await.vel = nv1_msg::hub::Movement {
                        x: 0.0,
                        y: 0.0,
                        angle: 0.0,
                    };

                    G_NEO_PIXEL_DATA.lock().await.jetson_connecting = false;

                    break;
                }
            }
        }
        match postcard::from_bytes_cobs::<nv1_msg::hub::ToHub>(&mut msg_with_cobs) {
            Ok(msg) => {
                // info!("Linear X: {}", msg.vel.x);
                // info!("Linear Y: {}", msg.vel.y);
                // info!("Angular Z: {}", msg.vel.angle);

                G_JETSON_RX.lock().await.vel = msg.vel;

                G_NEO_PIXEL_DATA.lock().await.jetson_connecting = true;
            }
            Err(_) => {
                error!("[UART Jetson] postcard decode error");
            }
        };

        let msg = G_JETSON_TX.lock().await.take();
        match postcard::to_vec_cobs::<nv1_msg::hub::ToJetson, 64>(&msg) {
            Ok(msg_with_cobs) => {
                match uart_tx.write(&msg_with_cobs).await {
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
    }
}

#[embassy_executor::task]
async fn ui_task(
    mut ui: HubUI<
        'static,
        Ssd1306<
            I2CInterface<I2c<'static, mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    >,
    mut gpio_ui_up: ExtiInput<'static>,
    mut gpio_ui_down: ExtiInput<'static>,
    mut gpio_ui_enter: ExtiInput<'static>,
) {
    loop {
        select3(
            gpio_ui_up.wait_for_any_edge(),
            gpio_ui_down.wait_for_any_edge(),
            gpio_ui_enter.wait_for_any_edge(),
        )
        .await;

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
        display.flush().unwrap();
    }
}

#[derive(Debug, Clone, Copy)]
struct NeoPixelData {
    pub jetson_connecting: bool,
    pub pause: bool,
    pub ball_dir: f32,
}

#[embassy_executor::task]
async fn neo_pixel_task(
    mut neo_pixel: NeoPixelPwm<peripherals::TIM4>,
    dma: &'static mut peripherals::DMA1_CH0,
) {
    const LED_COUNT: usize = 32;

    let mut neo_pixel_data = [RGB8::default(); LED_COUNT];
    for c in neo_pixel_data.iter_mut() {
        *c = RGB8 { r: 0, g: 0, b: 0 };
    }

    const SPREAD_PATTERN: [usize; 32] = [
        0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0,
        0, 0,
    ];

    let mut loop_count = 0;
    loop {
        let neo_pixel_info = G_NEO_PIXEL_DATA.lock().await.clone();

        if neo_pixel_info.pause {
            // windows loading
            let color = if neo_pixel_info.jetson_connecting {
                RGB8 { r: 0, g: 255, b: 0 }
            } else {
                RGB8 { r: 255, g: 0, b: 0 }
            };

            let base_index = loop_count % LED_COUNT;
            let spread = SPREAD_PATTERN[loop_count % 32];
            for j in 0..3 {
                let offset = spread * (j as isize - 1) as usize; // 左右に広がる動き
                let index = (base_index + offset) % LED_COUNT;
                neo_pixel_data[index] = color;
            }
        } else {
            // ball dir
            let ball_dir = neo_pixel_info.ball_dir;
            let ball_dir = if ball_dir < 0.0 {
                2.0 * PI + ball_dir
            } else {
                ball_dir
            };

            let ball_dir = (ball_dir / (2.0 * PI) * LED_COUNT as f32) as usize;
            neo_pixel_data[ball_dir] = RGB8 { r: 0, g: 0, b: 255 };
        }

        neo_pixel.set_colors(dma, &mut neo_pixel_data).await;

        loop_count = (loop_count + 1) % LED_COUNT;
        Timer::after(Duration::from_millis(30)).await;
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
struct Settings {
    pub line_strength: f32,
    pub opp_goal_color: GoalColor,
    pub opencv_goal_blue: nv1_msg::hub::HSV,
    pub opencv_goal_yellow: nv1_msg::hub::HSV,
}

fn flash_read(f: &mut Flash<'_, Blocking>) -> Result<Settings, embassy_stm32::flash::Error> {
    let mut buf = [0u8; 128];
    f.blocking_read(0x6_0000, &mut buf)?;

    info!("flash read: {:?}", buf);

    let decoded = match postcard::from_bytes(&buf) {
        Ok(d) => d,
        Err(_) => Default::default(),
    };

    Ok(decoded)
}

fn flash_write(
    f: &mut Flash<'_, Blocking>,
    settings: &Settings,
) -> Result<(), embassy_stm32::flash::Error> {
    let mut buf = [0u8; 128];
    postcard::to_slice(settings, &mut buf).unwrap();

    f.blocking_erase(0x6_0000, 0x6_0000 + 128 * 1024)?;
    f.blocking_write(0x6_0000, &buf)?;

    Ok(())
}
