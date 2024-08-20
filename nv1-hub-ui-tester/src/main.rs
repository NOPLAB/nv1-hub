#![cfg_attr(feature = "no_std", no_std)]
#![cfg_attr(feature = "no_std", no_main)]

#[cfg_attr(feature = "no_std", panic_handler)]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(not(feature = "no_std"))]
fn main() -> anyhow::Result<()> {
    use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
    use embedded_graphics_simulator::{
        sdl2::Keycode, BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent,
        Window,
    };
    use nv1_hub_ui::{EventKey, HubUi, HubUiEvent};

    println!("Hello, world!");

    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .build();
    let mut window = Window::new("SSD1306", &output_settings);

    let options = nv1_hub_ui::HubUiOptions {
        display_size: Size::new(128, 64),
        menu_vertical_num: 3,
    };
    let mut ui = HubUi::new(&mut display, options);
    let mut ui_event = HubUiEvent::None;

    'running: loop {
        let mut display = ui.update(&ui_event);
        window.update(&mut display);

        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                SimulatorEvent::KeyDown { keycode, .. } => {
                    println!("Key pressed: {:?}", keycode);

                    match keycode {
                        Keycode::Up => ui_event = HubUiEvent::KeyDown(EventKey::Up),
                        Keycode::Down => ui_event = HubUiEvent::KeyDown(EventKey::Down),
                        Keycode::Return => ui_event = HubUiEvent::KeyDown(EventKey::Enter),
                        _ => {}
                    }
                }
                _ => {
                    ui_event = HubUiEvent::None;
                }
            }
        }
    }

    Ok(())
}
