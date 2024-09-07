use libm::{cosf, sinf};

#[derive(Debug, Clone, Copy)]
pub struct OmniWheel {
    motor_angle: f32,
    wheel_r: f32,
    tread: f32,
}

impl OmniWheel {
    pub fn new(motor_angle: f32, wheel_r: f32, tread: f32) -> OmniWheel {
        OmniWheel {
            motor_angle,
            wheel_r,
            tread,
        }
    }

    //     float calcOmniWheelSpeed(float motorAngle, float wheel_r, float tread,
    // 		float angle, float speed_x, float speed_y, float speed_angle) {
    // 	return (-sinf((angle + motorAngle) * DEG2RAD) * cosf(angle * DEG2RAD)
    // 			* speed_x
    // 			+ cosf((angle + motorAngle) * DEG2RAD) * cosf(angle * DEG2RAD)
    // 					* speed_y + speed_angle * tread) / wheel_r;
    // }

    pub fn calculate(&self, linier_x: f32, linier_y: f32, angle: f32, angle_speed: f32) -> f32 {
        (-sinf(angle + self.motor_angle) * cosf(angle) * linier_x
            + cosf(angle + self.motor_angle) * cosf(angle) * linier_y
            + angle_speed * self.tread)
            / self.wheel_r
    }
}
