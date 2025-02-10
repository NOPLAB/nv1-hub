use alloc::boxed::Box;
use alloc::vec::Vec;
use embassy_stm32::adc::{Adc, AnyAdcChannel, Instance};
use embassy_stm32::gpio::{Output, Pin};

pub struct Multiplexer<'a, T>
where
    T: Instance,
{
    num: usize,
    multiplexer_pins: Vec<Output<'a>>,
    adc: Adc<'a, T>,
    adc_channel: Box<AnyAdcChannel<T>>,
}

impl<'a, T> Multiplexer<'a, T>
where
    T: Instance,
{
    pub fn new(
        num: usize,
        multiplexer_pins: Vec<Output<'a>>,
        adc: Adc<'a, T>,
        adc_channel: Box<AnyAdcChannel<T>>,
    ) -> Multiplexer<'a, T> {
        Multiplexer {
            num,
            multiplexer_pins,
            adc,
            adc_channel,
        }
    }

    pub fn read(&mut self) -> Vec<u16> {
        let mut values = Vec::new();

        for i in 0..self.num {
            for j in 0..self.num {
                let k = (1 << j & i) != 0;

                if k {
                    self.multiplexer_pins[j].set_high();
                } else {
                    self.multiplexer_pins[j].set_low();
                }
            }

            values.push(self.adc.blocking_read(self.adc_channel.as_mut()));
        }

        values
    }
}
