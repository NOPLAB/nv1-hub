use core::cell::RefCell;
use core::ffi::c_void;
use core::ptr::{null, null_mut};

use alloc::rc::Rc;
use defmt::{info, println};
use embassy_futures::block_on;
use embassy_stm32::gpio::{Input, Output, Pin};
use embassy_stm32::i2c::I2c;
use embassy_time::{Instant, Timer};
use sh2::sh2_ProductId_t;
use sh2::sh2_ProductIds_t;
use sh2::sh2_getProdIds;
use sh2::{
    sh2_AsyncEvent_t, sh2_Hal_t, sh2_RotationVector_t, sh2_SensorConfig, sh2_SensorEvent_t,
    sh2_SensorId_e_SH2_GAME_ROTATION_VECTOR, sh2_SensorId_e_SH2_GYRO_INTEGRATED_RV, sh2_SensorId_t,
    sh2_SensorValue__bindgen_ty_1, sh2_SensorValue_t, sh2_decodeSensorEvent, sh2_open, sh2_service,
    sh2_setSensorCallback, sh2_setSensorConfig, SH2_OK,
};

use crate::fmt::error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bno08xI2cAddress {
    A = 0x4A,
    B = 0x4B,
}

#[repr(C)]
#[no_mangle]
struct Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>
where
    Instance: embassy_stm32::i2c::Instance,
    TXDMA: embassy_stm32::i2c::TxDma<Instance>,
    RXDMA: embassy_stm32::i2c::RxDma<Instance>,
    IntPin: Pin,
{
    hal: sh2_Hal_t,
    i2c: Rc<RefCell<I2c<'a, Instance, TXDMA, RXDMA>>>,
    gpio_int: Rc<RefCell<Input<'a, IntPin>>>,
}

pub struct Bno08xI2c<'a, Instance, TXDMA, RXDMA, ResetPin, IntPin>
where
    Instance: embassy_stm32::i2c::Instance,
    TXDMA: embassy_stm32::i2c::TxDma<Instance>,
    RXDMA: embassy_stm32::i2c::RxDma<Instance>,
    ResetPin: Pin,
    IntPin: Pin,
{
    i2c: Rc<RefCell<I2c<'a, Instance, TXDMA, RXDMA>>>,
    gpio_reset: Output<'a, ResetPin>,
    gpio_int: Rc<RefCell<Input<'a, IntPin>>>,
    sensor_value_p: *mut sh2_SensorValue_t,
    pub sensor_value: sh2_SensorValue_t,
    hal: Option<Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>>,
}

impl<'a, Instance, TXDMA, RXDMA: 'a, ResetPin, IntPin>
    Bno08xI2c<'a, Instance, TXDMA, RXDMA, ResetPin, IntPin>
where
    Instance: embassy_stm32::i2c::Instance,
    TXDMA: embassy_stm32::i2c::TxDma<Instance>,
    RXDMA: embassy_stm32::i2c::RxDma<Instance>,
    ResetPin: Pin,
    IntPin: Pin,
{
    const BNO08X_ADDRESS: Bno08xI2cAddress = Bno08xI2cAddress::B;
    const MAXBUFFERSIZE: usize = 512;

    pub fn new(
        i2c: I2c<'a, Instance, TXDMA, RXDMA>,
        gpio_reset: Output<'a, ResetPin>,
        gpio_int: Input<'a, IntPin>,
    ) -> Self {
        let i2c = Rc::new(RefCell::new(i2c));

        Self {
            i2c,
            gpio_reset,
            gpio_int: Rc::new(RefCell::new(gpio_int)),
            sensor_value_p: null_mut(),
            sensor_value: sh2_SensorValue_t {
                timestamp: 0,
                sensorId: 0,
                sequence: 0,
                status: 0,
                delay: 0,
                un: sh2_SensorValue__bindgen_ty_1 {
                    gameRotationVector: sh2_RotationVector_t {
                        i: 0.0,
                        j: 0.0,
                        k: 0.0,
                        real: 0.0,
                    },
                },
            },
            hal: None,
        }
    }

    pub async fn begin(&mut self) -> bool {
        let hal = sh2_Hal_t {
            open: Some(Self::i2c_hal_open),
            close: Some(Self::i2c_hal_close),
            read: Some(Self::i2c_hal_read),
            write: Some(Self::i2c_hal_write),
            getTimeUs: Some(Self::i2c_hal_get_time_us),
        };

        let hal = Bno08xI2cSh2Hal {
            hal,
            i2c: self.i2c.clone(),
            gpio_int: self.gpio_int.clone(),
        };

        self.hal = Some(hal);

        self.sensor_value_p = &mut self.sensor_value as *mut sh2_SensorValue_t;

        return self.init(0).await;
    }

    pub fn get_sensor_event(&mut self) -> bool {
        unsafe { sh2_service() };

        self.sensor_value.timestamp = 0;

        if (unsafe { *self.sensor_value_p }).timestamp == 0
            && (unsafe { *self.sensor_value_p }).sensorId
                != sh2_SensorId_e_SH2_GYRO_INTEGRATED_RV as u8
        {
            // info!("No sensor event");
            info!("Sensor ID: {}", (unsafe { *self.sensor_value_p }).timestamp);
            return false;
        }

        true
    }

    fn enable_report(
        &self,
        sensor_id: sh2_SensorId_t,
        interval_us: u32,
        sensor_specific: u32,
    ) -> bool {
        let config = sh2_SensorConfig {
            batchInterval_us: 0,
            changeSensitivityEnabled: false,
            wakeupEnabled: false,
            changeSensitivityRelative: false,
            alwaysOnEnabled: false,
            changeSensitivity: 0,
            sniffEnabled: false,
            sensorSpecific: sensor_specific,
            reportInterval_us: interval_us,
        };

        let status = unsafe { sh2_setSensorConfig(sensor_id, &config) };

        if status as u32 != SH2_OK {
            return false;
        }

        true
    }

    extern "C" fn i2c_hal_open(s: *mut sh2_Hal_t) -> i32 {
        let hal = unsafe { &*(s as *mut Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>) };
        let mut i2c = hal.i2c.borrow_mut();

        while hal.gpio_int.borrow().is_low() {
            info!("Waiting INT pin");
        }

        let soft_reset_pkt = [5, 0, 1, 0, 1];
        for _ in 0..1000 {
            match i2c.blocking_write(Self::BNO08X_ADDRESS as u8, &soft_reset_pkt) {
                Ok(_) => {
                    info!("BNO08x - Soft reset successful");
                    return 0;
                }
                Err(err) => {
                    info!("BNO08x - Soft reset failed");
                    info!("Error: {:?}", err);
                }
            }
        }

        -1
    }

    extern "C" fn i2c_hal_close(s: *mut sh2_Hal_t) {
        todo!()
    }

    extern "C" fn i2c_hal_read(
        s: *mut sh2_Hal_t,
        mut buffer_p: *mut u8,
        len: u32,
        _t_us: *mut u32,
    ) -> i32 {
        let hal = unsafe { &*(s as *mut Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>) };
        let mut i2c = hal.i2c.borrow_mut();

        while hal.gpio_int.borrow().is_low() {
            info!("Waiting for INT pin");
        }

        let mut header = [0u8; 4];
        match i2c.blocking_read(Self::BNO08X_ADDRESS as u8, &mut header) {
            Ok(_) => {}
            Err(err) => {
                info!("Error reading header from I2C");
                info!("Error: {:?}", err);
                if err != embassy_stm32::i2c::Error::Nack {
                    return 0;
                }
            }
        }

        info!("Header: {:?}", header);

        let mut packet_size = (header[0] as u16) | ((header[1] as u16) << 8);
        packet_size &= !0x8000;

        let i2c_buffer_max = Self::MAXBUFFERSIZE;

        if packet_size > len as u16 {
            return 0;
        }

        let mut cargo_remaining = packet_size;
        let mut i2c_buffer = [0u8; Self::MAXBUFFERSIZE];
        let mut read_size;
        let mut cargo_read_amount;
        let mut first_read = true;

        info!("packet_size: {}", packet_size);

        let buffer_p_orig = buffer_p;

        while cargo_remaining > 0 {
            if first_read {
                read_size = core::cmp::min(i2c_buffer_max, cargo_remaining as usize);
            } else {
                read_size = core::cmp::min(i2c_buffer_max, cargo_remaining as usize + 4);
            }

            info!("read_size: {}", read_size);

            match i2c.blocking_read(Self::BNO08X_ADDRESS as u8, &mut i2c_buffer[..read_size]) {
                Ok(_) => {}
                Err(err) => {
                    info!("Error reading from I2C");
                    info!("Error: {:?}", err);
                    if err != embassy_stm32::i2c::Error::Nack {
                        return 0;
                    }
                }
            }

            if first_read {
                cargo_read_amount = read_size as u16;
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        i2c_buffer.as_ptr(),
                        buffer_p,
                        cargo_read_amount as usize,
                    )
                };
                first_read = false;
            } else {
                cargo_read_amount = read_size as u16 - 4;
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        i2c_buffer[4..].as_ptr(),
                        buffer_p,
                        cargo_read_amount as usize,
                    )
                };
            }

            buffer_p = unsafe { buffer_p.add(cargo_read_amount as usize) };
            cargo_remaining -= cargo_read_amount;
        }

        for i in 0..packet_size as usize {
            info!("{}: {}", i, unsafe { *buffer_p_orig.add(i) });
        }

        packet_size as i32
    }

    extern "C" fn i2c_hal_write(s: *mut sh2_Hal_t, buffer: *mut u8, len: u32) -> i32 {
        let hal = unsafe { &*(s as *mut Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>) };
        let mut i2c = hal.i2c.borrow_mut();

        info!("hal_write");

        let i2c_buffer_max = Self::MAXBUFFERSIZE;

        let write_size = core::cmp::min(i2c_buffer_max, len as usize);

        let buffer = unsafe { core::slice::from_raw_parts(buffer, write_size) };
        if i2c
            .blocking_write(Self::BNO08X_ADDRESS as u8, buffer)
            .is_err()
        {
            return 0;
        }

        write_size as i32
    }

    extern "C" fn i2c_hal_get_time_us(s: *mut sh2_Hal_t) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn init(&mut self, sensor_id: i32) -> bool {
        self.hal_hardware_reset().await;

        Timer::after_millis(100).await;

        let sh2_hal_p = self.hal.as_mut().unwrap()
            as *mut Bno08xI2cSh2Hal<'a, Instance, TXDMA, RXDMA, IntPin>
            as *mut sh2_Hal_t;

        let status = unsafe { sh2_open(sh2_hal_p, Some(Self::hal_callback), null_mut()) };
        if status != 0 {
            return false;
        }

        let mut prod_ids = sh2_ProductIds_t {
            entry: [sh2_ProductId_t {
                resetCause: 0,
                swVersionMajor: 0,
                swVersionMinor: 0,
                swPartNumber: 0,
                swBuildNumber: 0,
                swVersionPatch: 0,
                reserved0: 0,
                reserved1: 0,
            }; 5],
            numEntries: 0,
        };

        let status = unsafe { sh2_getProdIds(&mut prod_ids) };
        if status as u32 != SH2_OK {
            info!("Error getting product ids");
            info!("Status: {}", status);
            return false;
        }

        unsafe {
            sh2_setSensorCallback(
                Some(Self::sensor_handler),
                self as *mut Bno08xI2c<'a, Instance, TXDMA, RXDMA, ResetPin, IntPin> as *mut c_void,
            )
        };

        true
    }

    async fn hal_hardware_reset(&mut self) {
        self.gpio_reset.set_high();
        Timer::after_millis(20).await;
        self.gpio_reset.set_low();
        Timer::after_millis(20).await;
        self.gpio_reset.set_high();
        // Timer::after_millis(10).await;
    }

    extern "C" fn sensor_handler(cookie: *mut core::ffi::c_void, event: *mut sh2_SensorEvent_t) {
        let s = cookie as *mut Bno08xI2c<'a, Instance, TXDMA, RXDMA, ResetPin, IntPin>;

        let sensor_value_p = unsafe { (*s).sensor_value_p };
        let rc = unsafe { sh2_decodeSensorEvent(sensor_value_p, event) };
        if rc != 0 {
            println!("error decoding");
            unsafe { sensor_value_p.read().timestamp = 0 };
        }
    }

    extern "C" fn hal_callback(cookie: *mut core::ffi::c_void, p_event: *mut sh2_AsyncEvent_t) {
        // If we see a reset, set a flag so that sensors will be reconfigured.
        if unsafe { *p_event }.eventId == 0 {
            info!("RESET!");
            // _reset_occurred = true;
        }
    }

    pub fn enable_game_rotation_vector(&self, time_interval_us: u32) -> bool {
        self.enable_report(
            sh2_SensorId_e_SH2_GAME_ROTATION_VECTOR as u8,
            time_interval_us,
            0,
        )
    }

    pub fn get_game_rotation_vector(&self) -> sh2_RotationVector_t {
        unsafe { self.sensor_value.un.gameRotationVector }
    }
}
