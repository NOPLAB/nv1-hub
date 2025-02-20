use core::fmt::Display;

use alloc::format;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::Drawable;
use embedded_graphics::{
    mono_font::{MonoFont, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Size},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text as GText,
};
use num_traits::Num;

use crate::Event;

pub trait Element<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error>;
    fn event(&mut self, event: &Event, info: ElementInfo) -> bool;
}

pub struct ElementInfo {
    pub selected: bool,
    pub position: Point,
    pub size: Size,
}

#[derive(Debug, Clone, Copy)]
pub struct Text {
    pub text: &'static str,
    font: MonoFont<'static>,
}

impl Text {
    pub fn new(text: &'static str, font: MonoFont<'static>) -> Self {
        Text { text, font }
    }
}

impl<T> Element<T> for Text
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), <T as DrawTarget>::Error> {
        let position = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.font.character_size.width as i32 * self.text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.font.character_size.height as i32 / 4,
        );

        let character_style = MonoTextStyle::new(&self.font, BinaryColor::On);

        GText::new(self.text, position, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, _event: &Event, _info: ElementInfo) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Value<T, F> {
    pub title: &'static str,
    pub value: T,
    request: F,
    font: MonoFont<'static>,
}

impl<T, F> Value<T, F>
where
    T: Copy,
    F: FnMut(&mut T) -> (),
{
    pub fn new(title: &'static str, value: T, request: F, font: MonoFont<'static>) -> Self {
        Value {
            title,
            value,
            request,
            font,
        }
    }
}

impl<T, U, F> Element<T> for Value<U, F>
where
    T: DrawTarget<Color = BinaryColor>,
    U: Copy + Display,
    F: FnMut(&mut U) -> (),
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let text = format!("{}: {:<4.2}", self.title, self.value);

        let character_style = MonoTextStyle::new(&self.font, BinaryColor::On);

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.font.character_size.width as i32 * text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.font.character_size.height as i32 / 4,
        );

        GText::new(&text, text_pos, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, _event: &crate::Event, _info: ElementInfo) -> bool {
        (self.request)(&mut self.value);
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Button<F> {
    pub text: &'static str,
    pressed: bool,
    callback: F,
    font: MonoFont<'static>,
}

impl<F> Button<F>
where
    F: Fn(bool) -> (),
{
    pub fn new(text: &'static str, callback: F, font: MonoFont<'static>) -> Self {
        Button {
            text,
            pressed: false,
            callback,
            font,
        }
    }
}

impl<T, F> Element<T> for Button<F>
where
    T: DrawTarget<Color = BinaryColor>,
    F: FnMut(bool) -> (),
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let style = if self.pressed {
            PrimitiveStyle::with_fill(BinaryColor::On)
        } else {
            PrimitiveStyle::with_stroke(BinaryColor::On, 1)
        };

        Rectangle::new(info.position, info.size)
            .into_styled(style.clone())
            .draw(display)?;

        let character_style = if self.pressed {
            MonoTextStyle::new(&self.font, BinaryColor::Off)
        } else {
            MonoTextStyle::new(&self.font, BinaryColor::On)
        };

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.font.character_size.width as i32 * self.text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.font.character_size.height as i32 / 4,
        );

        GText::new(self.text, text_pos, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &crate::Event, info: ElementInfo) -> bool {
        match event {
            crate::Event::KeyDown(crate::EventKey::Enter) => {
                self.pressed = true;
            }
            _ => {
                self.pressed = false;
            }
        }

        if !info.selected {
            self.pressed = false;
        }

        if info.selected {
            (self.callback)(self.pressed);
        }

        false
    }
}

pub struct Slider<T, F> {
    pub value: T,
    pub min: T,
    pub max: T,
    pub one: T,
    entering: bool,
    callback: F,
    font: MonoFont<'static>,
}

impl<T, F> Slider<T, F>
where
    T: Num + Copy,
    F: Fn(T) -> (),
{
    pub fn new(value: T, min: T, max: T, one: T, callback: F, font: MonoFont<'static>) -> Self {
        Slider {
            value,
            min,
            max,
            one,
            entering: false,
            callback,
            font,
        }
    }

    pub fn set_value(&mut self, value: T) {
        self.value = value;
    }

    pub fn get_value(&self) -> T {
        self.value
    }
}

impl<T, U, F> Element<T> for Slider<U, F>
where
    T: DrawTarget<Color = BinaryColor>,
    U: Num + Copy + Display + PartialOrd,
    F: FnMut(U) -> (),
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let text = format!("{}", self.value);

        let character_style = MonoTextStyle::new(&self.font, BinaryColor::On);

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.font.character_size.width as i32 * text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.font.character_size.height as i32 / 4,
        );

        GText::new(&text, text_pos, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &crate::Event, _info: ElementInfo) -> bool {
        match event {
            crate::Event::KeyDown(crate::EventKey::Enter) => {
                self.entering = !self.entering;
            }
            crate::Event::KeyDown(crate::EventKey::Up) => {
                if self.entering && self.value < self.max {
                    self.value = self.value + self.one;
                    (self.callback)(self.value);
                }
            }
            crate::Event::KeyDown(crate::EventKey::Down) => {
                if self.entering && self.value > self.min {
                    self.value = self.value - self.one;
                    (self.callback)(self.value);
                }
            }
            _ => {}
        }

        self.entering
    }
}
