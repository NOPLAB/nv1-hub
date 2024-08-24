#![cfg_attr(feature = "no_std", no_std)]

extern crate alloc;

pub mod custom;
pub mod elements;
pub mod menu;

use core::fmt::Debug;

use alloc::boxed::Box;
use alloc::vec::Vec;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use menu::{Menu, MenuOption};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EventKey {
    Up,
    Down,
    Enter,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    None,
    KeyDown(EventKey),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HubUIOption {
    pub menu_option: MenuOption,
}

pub struct HubUI<'a, T> {
    option: HubUIOption,
    display: &'a mut T,
    menu: Vec<Box<dyn Menu<T>>>,
    menu_option: MenuOption,
}

impl<'a, T> HubUI<'a, T>
where
    T: DrawTarget<Color = BinaryColor>,
    <T as DrawTarget>::Error: Debug,
{
    pub fn new(display: &'a mut T, menu: Vec<Box<dyn Menu<T>>>, option: HubUIOption) -> Self {
        let option_c = option.clone();

        HubUI {
            option,
            display,
            menu,
            menu_option: option_c.menu_option,
        }
    }

    pub fn update(&mut self, event: &Event) -> &mut T {
        self.event(event);
        self.draw();
        return self.display;
    }

    fn draw(&mut self) -> &mut T {
        self.display.clear(BinaryColor::Off).unwrap();

        for m in self.menu.iter() {
            m.draw(self.display, &self.menu_option).unwrap();
        }

        return self.display;
    }

    fn event(&mut self, event: &Event) {
        for m in self.menu.iter_mut() {
            m.event(event, &self.menu_option);
        }
    }
}
