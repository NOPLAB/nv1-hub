#![cfg_attr(feature = "no_std", no_std)]

extern crate alloc;

pub mod elements;
pub mod menu;

use core::fmt::Debug;

use alloc::boxed::Box;
use alloc::vec::Vec;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use menu::{Menu, MenuOption};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKey {
    Up,
    Down,
    Enter,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    None,
    KeyDown(EventKey),
    KeyUp(EventKey),
}

#[derive(Debug, Clone, Copy)]
pub struct HubUIOption {
    pub menu_option: MenuOption,
}

pub struct HubUI<'a, T, M>
where
    T: DrawTarget<Color = BinaryColor>,
    M: Menu<T>,
{
    option: HubUIOption,
    display: &'a mut T,
    menu: Vec<Box<M>>,
}

impl<'a, T, M> HubUI<'a, T, M>
where
    T: DrawTarget<Color = BinaryColor>,
    <T as DrawTarget>::Error: Debug,
    M: Menu<T>,
{
    pub fn new(display: &'a mut T, menu: Vec<Box<M>>, option: HubUIOption) -> Self {
        HubUI {
            option,
            display,
            menu,
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
            m.draw(self.display, &self.option.menu_option).unwrap();
        }

        return self.display;
    }

    fn event(&mut self, event: &Event) {
        for m in self.menu.iter_mut() {
            m.event(event, &self.option.menu_option);
        }
    }
}

#[macro_export]
macro_rules! elements {
    ($t: ty, $( $x: expr ), *) => {
        {
            let mut elements: Vec<Box<dyn Element<$t>>> = Vec::new();
            $(
                elements.push(Box::new($x));
            ) *
            elements
        }
    };
}

#[macro_export]
macro_rules! menus {
    ($( $x: expr ), *) => {
        {
            let mut menus = Vec::new();
            $(
                menus.push(Box::new($x));
            ) *
            menus
        }
    };
}
