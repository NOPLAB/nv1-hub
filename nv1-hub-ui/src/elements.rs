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
pub struct Value<T> {
    pub option: ValueOption,
    pub title: &'static str,
    pub value: T,
}

impl<T> Value<T>
where
    T: Num + Copy,
{
    pub fn new(title: &'static str, value: T, option: ValueOption) -> Self {
        Value {
            option,
            title,
            value,
        }
    }

    pub fn set_value(&mut self, value: T) {
        self.value = value;
    }

    pub fn get_value(&self) -> T {
        self.value
    }
}

impl<T, U> Element<T> for Value<U>
where
    T: DrawTarget<Color = BinaryColor>,
    U: Num + Copy + Display,
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
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ButtonOption {
    pub font: MonoFont<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct Button {
    pub option: ButtonOption,
    pub text: &'static str,
    selected: bool,
}

impl Button {
    pub fn new(text: &'static str, option: ButtonOption) -> Self {
        Button {
            option,
            text,
            selected: false,
        }
    }

    pub fn is_pressed(&self) -> bool {
        self.selected
    }
}

impl<T> Element<T> for Button
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: ElementInfo) -> Result<(), T::Error> {
        let style = if self.selected {
            PrimitiveStyle::with_fill(BinaryColor::On)
        } else {
            PrimitiveStyle::with_stroke(BinaryColor::On, 1)
        };

        Rectangle::new(info.position, info.size)
            .into_styled(style.clone())
            .draw(display)?;

        let character_style = if self.selected {
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
                self.selected = true;
            }
            _ => {
                self.selected = false;
            }
        }

        if !info.selected {
            self.selected = false;
        }

        false
    }
}

pub struct SliderOption {
    pub font: MonoFont<'static>,
}

pub struct Slider<T> {
    pub option: SliderOption,
    pub value: T,
    pub min: T,
    pub max: T,
    pub one: T,
    entering: bool,
}

impl<T> Slider<T>
where
    T: Num + Copy,
{
    pub fn new(value: T, min: T, max: T, one: T, option: SliderOption) -> Self {
        Slider {
            option,
            value,
            min,
            max,
            one,
            entering: false,
        }
    }

    pub fn set_value(&mut self, value: T) {
        self.value = value;
    }

    pub fn get_value(&self) -> T {
        self.value
    }
}

impl<T, U> Element<T> for Slider<U>
where
    T: DrawTarget<Color = BinaryColor>,
    U: Num + Copy + Display + PartialOrd,
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
                }
            }
            crate::Event::KeyDown(crate::EventKey::Down) => {
                if self.entering && self.value > self.min {
                    self.value = self.value - self.one;
                }
            }
            _ => {}
        }

        self.entering
    }
}
