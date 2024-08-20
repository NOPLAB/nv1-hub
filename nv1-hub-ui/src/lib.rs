#![cfg_attr(feature = "no_std", no_std)]

use core::fmt::Debug;

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::{DrawTarget, Size};
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Point, Primitive},
    primitives::*,
};

pub enum EventKey {
    Up,
    Down,
    Enter,
}

pub enum HubUiEvent {
    None,
    KeyDown(EventKey),
}

pub struct HubUiOptions {
    pub display_size: Size,
    pub menu_vertical_num: u8,
}

pub struct HubUi<'a, T> {
    display: &'a mut T,
    options: HubUiOptions,
    menu_index: u8,
    element_index: u8,
}

impl<'a, T> HubUi<'a, T>
where
    T: DrawTarget<Color = BinaryColor>,
    <T as DrawTarget>::Error: Debug,
{
    pub fn new(display: &'a mut T, options: HubUiOptions) -> Self {
        HubUi {
            display,
            options,
            menu_index: 0,
            element_index: 0,
        }
    }

    pub fn update(&mut self, event: &HubUiEvent) -> &mut T {
        self.event(event);
        self.draw();
        return self.display;
    }

    fn draw(&mut self) -> &mut T {
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        self.display.clear(BinaryColor::Off).unwrap();

        Circle::new(Point::new(72, 8), 48)
            .into_styled(line_style)
            .draw(self.display)
            .unwrap();

        Line::new(Point::new(48, 16), Point::new(8, 16))
            .into_styled(line_style)
            .draw(self.display)
            .unwrap();

        Line::new(Point::new(48, 16), Point::new(64, 32))
            .into_styled(line_style)
            .draw(self.display)
            .unwrap();

        Rectangle::new(Point::new(79, 15), Size::new(34, 34))
            .into_styled(line_style)
            .draw(self.display)
            .unwrap();

        let text_style = MonoTextStyle::new(
            &embedded_graphics::mono_font::ascii::FONT_6X9,
            BinaryColor::On,
        );
        Text::new("Hello World!", Point::new(5, 5), text_style)
            .draw(self.display)
            .unwrap();

        return self.display;
    }

    fn event(&mut self, event: &HubUiEvent) {
        match event {
            HubUiEvent::KeyDown(key) => match key {
                EventKey::Up => {}
                EventKey::Down => {}
                EventKey::Enter => {}
            },
            HubUiEvent::None => {}
        }
    }
}
