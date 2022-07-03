#![no_main]
/// This is a modified version of the 'example' project optimized for fuzzing.

use libfuzzer_sys::fuzz_target;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal_fuzz as hal_fuzz;
use bno055::{BNO055OperationMode, Bno055};

struct Delay {}

impl Delay { pub fn new() -> Self { Delay{ } }}

impl DelayMs<u16> for Delay {
   fn delay_ms(&mut self, _ms: u16) {
       // no-op, go as fast as possible for fuzzing
   }
}

type I2cError = ();

fuzz_target!(|data: &[u8]| {
    use hal_fuzz::shared_data::FuzzData;
    let data = FuzzData::new(data);
    let i2c: hal_fuzz::i2c::I2cFuzz<'_, I2cError> = hal_fuzz::i2c::I2cFuzz::new(data);
    let mut delay = Delay::new();

    let mut imu = Bno055::new(i2c).with_alternative_address();

    let _ = imu.init(&mut delay);

    let _ = imu.set_mode(BNO055OperationMode::NDOF, &mut delay);

    let _ = imu.get_calibration_status();

    if !imu.is_fully_calibrated().is_ok() {
        let _ = imu.get_calibration_status();
    }

    if let Ok(calib) = imu.calibration_profile(&mut delay) {
        let _ = imu.set_calibration_profile(calib, &mut delay);
    }

    // Quaternion; due to a bug in the BNO055, this is recommended over Euler Angles
    let _quaternion = imu.quaternion();

    // Euler angles, directly read
    let _euler =  imu.euler_angles();

    // Temperature.
    let _temp = imu.temperature();
});
