use embassy_stm32::{
    time::Hertz,
    timer::{simple_pwm::SimplePwm, Ch1Dma, GeneralInstance4Channel},
};
use rgb::RGB8;

const NEO_PIXEL_NUM: usize = 32;

pub struct NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pwm: SimplePwm<'static, T>,
    one_duty: u16,
    zero_duty: u16,
    brightness: u8,
}

impl<T> NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'static, T>, pwm_hz: Hertz) -> Self {
        let max_duty = pwm.max_duty_cycle();

        let one_duty: u16 = max_duty / 2;
        let zero_duty: u16 = (max_duty as f32 * (260.0e-6 * (pwm_hz.0 / 1000) as f32)) as u16;

        Self {
            pwm,
            one_duty,
            zero_duty,
            brightness: 45,
        }
    }

    async fn write(&mut self, dma: &mut impl Ch1Dma<T>, colors: &mut [RGB8]) {
        let mut duty = [0u16; 24 * NEO_PIXEL_NUM];

        for (n, color) in colors.iter_mut().enumerate() {
            for i in 0..8 {
                if color.g & 0b1000_0000 != 0 {
                    duty[n * 24 + i] = self.one_duty;
                } else {
                    duty[n * 24 + i] = self.zero_duty;
                }

                if color.r & 0b1000_0000 != 0 {
                    duty[n * 24 + i + 8] = self.one_duty;
                } else {
                    duty[n * 24 + i + 8] = self.zero_duty;
                }

                if color.b & 0b1000_0000 != 0 {
                    duty[n * 24 + i + 16] = self.one_duty;
                } else {
                    duty[n * 24 + i + 16] = self.zero_duty;
                }

                color.g <<= 1;
                color.r <<= 1;
                color.b <<= 1;
            }
        }

        self.pwm.ch1().enable();
        self.pwm.waveform_ch1(dma, &duty).await;
        self.pwm.ch1().disable();
    }

    pub async fn set_colors(&mut self, dma: &mut impl Ch1Dma<T>, colors: &mut [RGB8]) {
        let angle = 90.0 - self.brightness as f32;
        let angle = angle * core::f32::consts::PI / 180.0;

        let tan_angle = libm::tanf(angle);

        for color in &mut *colors {
            let r = color.r as f32;
            let g = color.g as f32;
            let b = color.b as f32;

            let r = r * tan_angle;
            let g = g * tan_angle;
            let b = b * tan_angle;

            color.r = r as u8;
            color.g = g as u8;
            color.b = b as u8;
        }

        self.write(dma, colors).await;
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        if brightness > 45 {
            self.brightness = 45;
        } else {
            self.brightness = brightness;
        }
    }
}
