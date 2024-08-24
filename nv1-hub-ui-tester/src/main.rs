#![cfg_attr(feature = "no_std", no_std)]
#![cfg_attr(feature = "no_std", no_main)]

#[cfg_attr(feature = "no_std", panic_handler)]
#[cfg(feature = "no_std")]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(not(feature = "no_std"))]
fn main() -> anyhow::Result<()> {
    use std::vec;

    use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
    use embedded_graphics_simulator::{
        sdl2::Keycode, BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent,
        Window,
    };
    use nv1_hub_ui::{
        elements::Element,
        menu::{ListMenu, ListMenuOption, Menu, MenuOption},
        Event, EventKey, HubUIOption,
    };

    println!("Hello, world!");

    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .build();
    let mut window = Window::new("SSD1306", &output_settings);

    let elements: Vec<Box<dyn Element<SimulatorDisplay<BinaryColor>>>> = vec![
        Box::new(nv1_hub_ui::elements::Button::new(
            "Button 1",
            nv1_hub_ui::elements::ButtonOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
        Box::new(nv1_hub_ui::elements::Button::new(
            "Button 2",
            nv1_hub_ui::elements::ButtonOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
        Box::new(nv1_hub_ui::elements::Button::new(
            "Button 3",
            nv1_hub_ui::elements::ButtonOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
        Box::new(nv1_hub_ui::elements::Button::new(
            "Button 4",
            nv1_hub_ui::elements::ButtonOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
        Box::new(nv1_hub_ui::elements::Text::new(
            "Hello",
            nv1_hub_ui::elements::TextOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
        Box::new(nv1_hub_ui::elements::Slider::new(
            0,
            -50,
            50,
            1,
            nv1_hub_ui::elements::SliderOption {
                font: embedded_graphics::mono_font::ascii::FONT_6X10,
            },
        )),
    ];
    let menu: Vec<Box<dyn Menu<SimulatorDisplay<BinaryColor>>>> = vec![Box::new(ListMenu::new(
        elements,
        ListMenuOption {
            vertical_num: 4,
            element_margin: 1,
            cursor_line_len: 4,
        },
    ))];

    let options = HubUIOption {
        menu_option: MenuOption {
            position: Point::new(2 + 64, 2),
            size: Size::new(62, 62),
        },
    };
    let mut ui = nv1_hub_ui::HubUI::new(&mut display, menu, options);
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
