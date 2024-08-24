use alloc::{boxed::Box, vec::Vec};

use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::Line;
use embedded_graphics::Drawable;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Size},
    primitives::{PrimitiveStyle, Rectangle},
};

use crate::elements::{Element, ElementInfo};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MenuOption {
    pub position: Point,
    pub size: Size,
}

pub trait Menu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: &MenuOption) -> Result<(), T::Error>;
    fn event(&mut self, event: &crate::Event, info: &MenuOption);
}

pub struct ListMenuOption {
    pub vertical_num: usize,
    pub element_margin: usize,
    pub cursor_line_len: i32,
}

pub struct ListMenu<T> {
    option: ListMenuOption,
    pub elements: Vec<Box<dyn Element<T>>>,
    selected_element: usize,
    scroll: usize,
    entering_cursor: bool,
}

impl<T> ListMenu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    pub fn new(elements: Vec<Box<dyn Element<T>>>, option: ListMenuOption) -> Self {
        ListMenu {
            option,
            elements,
            selected_element: 0,
            scroll: 0,
            entering_cursor: false,
        }
    }

    fn draw_cursor(&self, display: &mut T, info: &MenuOption) -> Result<(), T::Error> {
        let mut position = self.calculate_element_position(info, self.selected_element);
        position.x -= self.option.element_margin as i32;
        position.y -= self.option.element_margin as i32;

        let mut size = self.calculate_element_size(info);
        size.width += self.option.element_margin as u32 * 2;
        size.height += self.option.element_margin as u32 * 2;

        let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        if self.entering_cursor {
            Rectangle::new(position, size)
                .into_styled(style.clone())
                .draw(display)?;
            return Ok(());
        }

        let position_left_top = position;
        let position_left_top1 = Point::new(
            position_left_top.x + self.option.cursor_line_len,
            position_left_top.y,
        );
        let position_left_top2 = Point::new(
            position_left_top.x,
            position_left_top.y + self.option.cursor_line_len,
        );
        Line::new(position_left_top, position_left_top1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_left_top, position_left_top2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_right_top = Point::new(position.x + size.width as i32 - 1, position.y);
        let position_right_top1 = Point::new(
            position_right_top.x - self.option.cursor_line_len,
            position_right_top.y,
        );
        let position_right_top2 = Point::new(
            position_right_top.x,
            position_right_top.y + self.option.cursor_line_len,
        );
        Line::new(position_right_top, position_right_top1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_right_top, position_right_top2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_left_bottom = Point::new(position.x, position.y + size.height as i32 - 1);
        let position_left_bottom1 = Point::new(
            position_left_bottom.x + self.option.cursor_line_len,
            position_left_bottom.y,
        );
        let position_left_bottom2 = Point::new(
            position_left_bottom.x,
            position_left_bottom.y - self.option.cursor_line_len,
        );
        Line::new(position_left_bottom, position_left_bottom1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_left_bottom, position_left_bottom2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_right_bottom = Point::new(
            position.x + size.width as i32 - 1,
            position.y + size.height as i32 - 1,
        );
        let position_right_bottom1 = Point::new(
            position_right_bottom.x - self.option.cursor_line_len,
            position_right_bottom.y,
        );
        let position_right_bottom2 = Point::new(
            position_right_bottom.x,
            position_right_bottom.y - self.option.cursor_line_len,
        );
        Line::new(position_right_bottom, position_right_bottom1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_right_bottom, position_right_bottom2)
            .into_styled(style.clone())
            .draw(display)?;

        Ok(())
    }

    fn calculate_element_position(&self, info: &MenuOption, index: usize) -> Point {
        let height = info.size.height / self.option.vertical_num as u32;

        Point::new(
            info.position.x,
            info.position.y + height as i32 * (index as i32 - self.scroll as i32),
        )
    }

    fn calculate_element_size(&self, info: &MenuOption) -> Size {
        Size::new(
            info.size.width - self.option.element_margin as u32 * 2,
            info.size.height / self.option.vertical_num as u32
                - self.option.element_margin as u32 * 2,
        )
    }
}

impl<T> Menu<T> for ListMenu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T, info: &MenuOption) -> Result<(), T::Error> {
        for (i, element) in self.elements.iter().enumerate() {
            let position = self.calculate_element_position(info, i);
            let size = self.calculate_element_size(info);

            let info = ElementInfo {
                selected: i == self.selected_element,
                position,
                size,
            };
            element.draw(display, info)?;
        }

        self.draw_cursor(display, info)?;
        Ok(())
    }

    fn event(&mut self, event: &crate::Event, info: &MenuOption) {
        let positions: Vec<_> = (0..self.elements.len())
            .map(|i| self.calculate_element_position(info, i))
            .collect();

        let size = self.calculate_element_size(info);

        let mut entering = false;

        for (i, element) in self.elements.iter_mut().enumerate() {
            let position = positions[i];
            let info = ElementInfo {
                selected: i == self.selected_element,
                position,
                size,
            };

            let result = element.event(event, info);

            if i == self.selected_element {
                entering = result;
            }
        }

        self.entering_cursor = entering;

        if entering {
            return;
        } else {
            match event {
                crate::Event::KeyDown(crate::EventKey::Up) => {
                    if self.selected_element > 0 {
                        self.selected_element -= 1;
                    }
                    if self.selected_element < self.scroll {
                        self.scroll -= 1;
                    }
                }
                crate::Event::KeyDown(crate::EventKey::Down) => {
                    if self.selected_element < self.elements.len() - 1 {
                        self.selected_element += 1;
                    }
                    if self.selected_element >= self.option.vertical_num + self.scroll {
                        self.scroll += 1;
                    }
                }
                crate::Event::KeyDown(crate::EventKey::Enter) => {}
                _ => {}
            }
        }
    }
}
