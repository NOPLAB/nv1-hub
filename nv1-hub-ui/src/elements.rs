use core::fmt::{write, Display};

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

pub struct ElementInfo {
    pub selected: bool,
    pub position: Point,
    pub size: Size,
}

pub trait Element<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error>;
    fn event(&mut self, event: &Event, info: ElementInfo) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub struct TextOption {
    pub font: MonoFont<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct Text {
    pub option: TextOption,
    pub text: &'static str,
}

impl Text {
    pub fn new(text: &'static str, option: TextOption) -> Self {
        Text { option, text }
    }
}

impl<T> Element<T> for Text
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), <T as DrawTarget>::Error> {
        let position = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.option.font.character_size.width as i32 * self.text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.option.font.character_size.height as i32 / 4,
        );

        let character_style = MonoTextStyle::new(&self.option.font, BinaryColor::On);

        GText::new(self.text, position, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &Event, info: ElementInfo) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ValueOption {
    pub font: MonoFont<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct Value<T, F> {
    pub option: ValueOption,
    pub title: &'static str,
    pub value: T,
    request: F,
}

impl<T, F> Value<T, F>
where
    T: Num + Copy,
    F: Fn(&mut T) -> (),
{
    pub fn new(title: &'static str, value: T, request: F, option: ValueOption) -> Self {
        Value {
            option,
            title,
            value,
            request,
        }
    }
}

impl<T, U, F> Element<T> for Value<U, F>
where
    T: DrawTarget<Color = BinaryColor>,
    U: Num + Copy + Display,
    F: Fn(&mut U) -> (),
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let text = format!("{}: {}", self.title, self.value);

        let character_style = MonoTextStyle::new(&self.option.font, BinaryColor::On);

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.option.font.character_size.width as i32 * text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.option.font.character_size.height as i32 / 4,
        );

        GText::new(&text, text_pos, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &crate::Event, info: ElementInfo) -> bool {
        (self.request)(&mut self.value);
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ButtonOption {
    pub font: MonoFont<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct Button<F> {
    pub option: ButtonOption,
    pub text: &'static str,
    pressed: bool,
    callback: F,
}

impl<F> Button<F>
where
    F: Fn(bool) -> (),
{
    pub fn new(text: &'static str, callback: F, option: ButtonOption) -> Self {
        Button {
            option,
            text,
            pressed: false,
            callback,
        }
    }
}

impl<T, F> Element<T> for Button<F>
where
    T: DrawTarget<Color = BinaryColor>,
    F: Fn(bool) -> (),
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
            MonoTextStyle::new(&self.option.font, BinaryColor::Off)
        } else {
            MonoTextStyle::new(&self.option.font, BinaryColor::On)
        };

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.option.font.character_size.width as i32 * self.text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.option.font.character_size.height as i32 / 4,
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

pub struct SliderOption {
    pub font: MonoFont<'static>,
}

pub struct Slider<T, F> {
    pub option: SliderOption,
    pub value: T,
    pub min: T,
    pub max: T,
    pub one: T,
    entering: bool,
    callback: F,
}

impl<T, F> Slider<T, F>
where
    T: Num + Copy,
    F: Fn(T) -> (),
{
    pub fn new(value: T, min: T, max: T, one: T, callback: F, option: SliderOption) -> Self {
        Slider {
            option,
            value,
            min,
            max,
            one,
            entering: false,
            callback,
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
    F: Fn(U) -> (),
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let text = format!("{}", self.value);

        let character_style = MonoTextStyle::new(&self.option.font, BinaryColor::On);

        let text_pos = Point::new(
            info.position.x + info.size.width as i32 / 2
                - self.option.font.character_size.width as i32 * text.len() as i32 / 2,
            info.position.y
                + info.size.height as i32 / 2
                + self.option.font.character_size.height as i32 / 4,
        );

        GText::new(&text, text_pos, character_style).draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &crate::Event, info: ElementInfo) -> bool {
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
