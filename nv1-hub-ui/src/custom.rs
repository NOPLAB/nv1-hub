use embedded_graphics::{pixelcolor::BinaryColor, prelude::DrawTarget};

use crate::menu::{Menu, MenuOption};

pub struct NV1Menu {
    option: MenuOption,
}

impl NV1Menu {
    pub fn new(option: MenuOption) -> Self {
        NV1Menu { option }
    }
}

impl<T> Menu<T> for NV1Menu
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: &MenuOption) -> Result<(), T::Error> {
        todo!()
    }

    fn event(&mut self, event: &crate::Event, info: &MenuOption) {
        todo!()
    }
}
