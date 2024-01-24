#![no_main]
/// This is a modified version of the 'example' project optimized for fuzzing.

use libfuzzer_sys::fuzz_target;

use embedded_hal::{delay::DelayNs, i2c::SevenBitAddress};
use embedded_hal_fuzz::i2c::ArbitraryI2c;

use bno055::{BNO055OperationMode, Bno055};

struct Delay {}

impl Delay { pub fn new() -> Self { Delay{ } }}

impl DelayNs for Delay {
   fn delay_ns(&mut self, _ns: u32) {
       // no-op, go as fast as possible for fuzzing
   }
}


fuzz_target!(|i2c: ArbitraryI2c<SevenBitAddress>| {
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
