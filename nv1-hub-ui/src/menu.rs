use alloc::{boxed::Box, vec::Vec};

use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{Circle, CornerRadii, Line, RoundedRectangle};
use embedded_graphics::Drawable;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Size},
    primitives::{PrimitiveStyle, Rectangle},
};

use crate::elements::{Element, ElementInfo};

pub trait Menu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), T::Error>;
    fn event(&mut self, event: &crate::Event);
}

pub struct ListMenuOption {
    pub position: Point,
    pub size: Size,
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

    fn draw_cursor(&self, display: &mut T) -> Result<(), T::Error> {
        let mut position = self.calculate_element_position(self.selected_element);
        position.x -= self.option.element_margin as i32;
        position.y -= self.option.element_margin as i32;

        let mut size = self.calculate_element_size();
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

    fn calculate_element_position(&self, index: usize) -> Point {
        let height = self.option.size.height / self.option.vertical_num as u32;

        Point::new(
            self.option.position.x,
            self.option.position.y + height as i32 * (index as i32 - self.scroll as i32),
        )
    }

    fn calculate_element_size(&self) -> Size {
        Size::new(
            self.option.size.width - self.option.element_margin as u32 * 2,
            self.option.size.height / self.option.vertical_num as u32
                - self.option.element_margin as u32 * 2,
        )
    }
}

impl<T> Menu<T> for ListMenu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), T::Error> {
        for (i, element) in self.elements.iter().enumerate() {
            let position = self.calculate_element_position(i);
            let size = self.calculate_element_size();

            let info = ElementInfo {
                selected: i == self.selected_element,
                position,
                size,
            };
            element.draw(display, info)?;
        }

        self.draw_cursor(display)?;
        Ok(())
    }

    fn event(&mut self, event: &crate::Event) {
        let positions: Vec<_> = (0..self.elements.len())
            .map(|i| self.calculate_element_position(i))
            .collect();

        let size = self.calculate_element_size();

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

pub struct RobotStatusMenuOption {
    pub position: Point,
    pub size: Size,
}

pub struct RobotStatusMenu {
    option: RobotStatusMenuOption,
}

impl RobotStatusMenu {
    pub fn new(option: RobotStatusMenuOption) -> Self {
        RobotStatusMenu { option }
    }
}

impl<T> Menu<T> for RobotStatusMenu
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), <T as DrawTarget>::Error> {
        let max_size = self.option.size.width.min(self.option.size.height);

        let circle_style = PrimitiveStyle::with_stroke(BinaryColor::On, 2);
        let circle = Circle::new(
            Point::new(self.option.position.x, self.option.position.y),
            max_size,
        );
        circle.into_styled(circle_style).draw(display)?;
        let circle_center = circle.center();

        let wheel_position_radius = max_size / 2 - 8;
        let wheel_position_size_offset = 8;

        let wheel_style = PrimitiveStyle::with_stroke(BinaryColor::On, 8);

        let w_top_left_start = Point::new(
            circle_center.x - wheel_position_radius as i32 - wheel_position_size_offset as i32,
            circle_center.y - wheel_position_radius as i32 + wheel_position_size_offset as i32,
        );
        let w_top_left_end = Point::new(
            circle_center.x - wheel_position_radius as i32 + wheel_position_size_offset as i32,
            circle_center.y - wheel_position_radius as i32 - wheel_position_size_offset as i32,
        );
        Line::new(w_top_left_start, w_top_left_end)
            .into_styled(wheel_style)
            .draw(display)?;

        let w_top_right_start = Point::new(
            circle_center.x + wheel_position_radius as i32 - wheel_position_size_offset as i32,
            circle_center.y - wheel_position_radius as i32 - wheel_position_size_offset as i32,
        );
        let w_top_right_end = Point::new(
            circle_center.x + wheel_position_radius as i32 + wheel_position_size_offset as i32,
            circle_center.y - wheel_position_radius as i32 + wheel_position_size_offset as i32,
        );
        Line::new(w_top_right_start, w_top_right_end)
            .into_styled(wheel_style)
            .draw(display)?;

        let w_bottom_left_start = Point::new(
            circle_center.x - wheel_position_radius as i32 - wheel_position_size_offset as i32,
            circle_center.y + wheel_position_radius as i32 - wheel_position_size_offset as i32,
        );
        let w_bottom_left_end = Point::new(
            circle_center.x - wheel_position_radius as i32 + wheel_position_size_offset as i32,
            circle_center.y + wheel_position_radius as i32 + wheel_position_size_offset as i32,
        );
        Line::new(w_bottom_left_start, w_bottom_left_end)
            .into_styled(wheel_style)
            .draw(display)?;

        let w_bottom_right_start = Point::new(
            circle_center.x + wheel_position_radius as i32 - wheel_position_size_offset as i32,
            circle_center.y + wheel_position_radius as i32 + wheel_position_size_offset as i32,
        );
        let w_bottom_right_end = Point::new(
            circle_center.x + wheel_position_radius as i32 + wheel_position_size_offset as i32,
            circle_center.y + wheel_position_radius as i32 - wheel_position_size_offset as i32,
        );
        Line::new(w_bottom_right_start, w_bottom_right_end)
            .into_styled(wheel_style)
            .draw(display)?;

        Ok(())
    }

    fn event(&mut self, event: &crate::Event) {}
}
