#![no_main]
use libfuzzer_sys::fuzz_target;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal_fuzz as hal_fuzz;

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

    // Init BNO055 IMU
    let mut imu = bno055::Bno055::new(i2c);

    // Discard the result as we only care about it it crashes not if there
    // is an error.
    let _ = imu.init(&mut delay);
});
