#![cfg_attr(feature = "no_std", no_std)]
#![cfg_attr(feature = "no_std", no_main)]

use core::cell::RefCell;
use std::sync::Mutex;

#[cfg_attr(feature = "no_std", panic_handler)]
#[cfg(feature = "no_std")]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

static G_SHUTDOWN: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

static G_REBOOT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[cfg(not(feature = "no_std"))]
fn main() -> anyhow::Result<()> {
    use std::vec;

    use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
    use embedded_graphics_simulator::{
        sdl2::Keycode, BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent,
        Window,
    };
    use nv1_hub_ui::{
        elements::{Button, Element, Text, Value},
        menu::{ListMenu, ListMenuOption, MenuOption},
        Event, EventKey, HubUI, HubUIOption,
    };

    println!("Hello, world!");

    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .build();
    let mut window = Window::new("SSD1306", &output_settings);

    let ui_option = HubUIOption {
        menu_option: MenuOption {
            position: Point::new(2 + 64, 2),
            size: Size::new(64 - 4, 64 - 4),
        },
    };

    let ui_text = Text::new("Interface", embedded_graphics::mono_font::ascii::FONT_6X10);

    let ui_shutdown = Button::new(
        "Shutdown",
        |pressed| {
            G_SHUTDOWN.lock().as_mut().unwrap().replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let ui_reboot = Button::new(
        "Reboot",
        |pressed| {
            G_REBOOT.lock().as_mut().unwrap().replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let ui_line = Value::new(
        "L",
        0,
        |_| {},
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let elements: Vec<Box<dyn Element<SimulatorDisplay<BinaryColor>>>> = vec![
        Box::new(ui_text),
        Box::new(ui_shutdown),
        Box::new(ui_reboot),
        Box::new(ui_line),
    ];
    let menu = vec![Box::new(ListMenu::new(
        elements,
        ListMenuOption {
            vertical_num: 4,
            element_margin: 1,
            cursor_line_len: 4,
        },
    ))];
    let mut ui = HubUI::new(&mut display, menu, ui_option);

    let mut ui_event = Event::None;

    'running: loop {
        let mut display = ui.update(&ui_event);
        window.update(&mut display);

        ui_event = Event::None;
        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                SimulatorEvent::KeyDown { keycode, .. } => {
                    println!("Key pressed: {:?}", keycode);

                    ui_event = match keycode {
                        Keycode::Up => Event::KeyDown(EventKey::Up),
                        Keycode::Down => Event::KeyDown(EventKey::Down),
                        Keycode::Return => Event::KeyDown(EventKey::Enter),
                        _ => Event::None,
                    };
                }
                _ => {
                    ui_event = Event::None;
                }
            }
        }
    }

    Ok(())
}
