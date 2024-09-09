use libm::{cosf, sinf};

#[derive(Debug, Clone, Copy)]
pub struct OmniWheel {
    wheel_angle: f32,
    wheel_r: f32,
    tread: f32,
}

impl OmniWheel {
    pub fn new(wheel_angle: f32, wheel_r: f32, tread: f32) -> OmniWheel {
        OmniWheel {
            wheel_angle,
            wheel_r,
            tread,
        }
    }

    pub fn calculate(&self, linier_x: f32, linier_y: f32, angle: f32, angle_speed: f32) -> f32 {
        (-sinf(angle + self.wheel_angle) * cosf(angle) * linier_x
            + cosf(angle + self.wheel_angle) * cosf(angle) * linier_y
            + angle_speed * self.tread)
            / self.wheel_r
    }
}
