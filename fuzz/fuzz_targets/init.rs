#![no_main]
use libfuzzer_sys::fuzz_target;

use embedded_hal::{delay::DelayNs, i2c::SevenBitAddress};
use embedded_hal_fuzz::i2c::ArbitraryI2c;

struct Delay {}

impl Delay {
    pub fn new() -> Self {
        Delay {}
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, _ns: u32) {
        // no-op, go as fast as possible for fuzzing
    }
}

fuzz_target!(|i2c: ArbitraryI2c<SevenBitAddress>| {
    let mut delay = Delay::new();

    // Init BNO055 IMU
    let mut imu = bno055::Bno055::new(i2c);

    // Discard the result as we only care about it it crashes not if there
    // is an error.
    let _ = imu.init(&mut delay);
});
