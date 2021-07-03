use bno055::{BNO055OperationMode, Bno055};
use linux_embedded_hal::{Delay, I2cdev};
use mint::{EulerAngles, Quaternion};

fn main() {
    let dev = I2cdev::new("/dev/i2c-0").unwrap();
    let mut delay = Delay {};
    let mut imu = Bno055::new(dev).with_alternative_address();
    imu.init(&mut delay).expect("An error occurred while building the IMU");

    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("An error occurred while setting the IMU mode");

    let mut status = imu.get_calibration_status().unwrap();
    println!("The IMU's calibration status is: {:?}", status);

    // Wait for device to auto-calibrate.
    // Please perform steps necessary for auto-calibration to kick in.
    // Required steps are described in Datasheet section 3.11
    // Page 51, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf (As of 2021-07-02)
    println!("- About to begin BNO055 IMU calibration...");
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(1000));
        println!("Calibration status: {:?}", status);
    }

    let calib = imu.calibration_profile(&mut delay).unwrap();

    imu.set_calibration_profile(calib, &mut delay).unwrap();
    println!("       - Calibration complete!");

    // These are sensor fusion reading using the mint crate that the state will be read into
    let mut euler_angles: EulerAngles<f32, ()>; // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    let mut quaternion: Quaternion<f32>; // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);

    loop {
        // Quaternion; due to a bug in the BNO055, this is recommended over Euler Angles
        match imu.quaternion() {
            Ok(val) => {
                quaternion = val;
                println!("IMU Quaternion: {:?}", quaternion);
                std::thread::sleep(std::time::Duration::from_millis(500));
            }
            Err(e) => {
                eprintln!("{:?}", e);
            }
        }

        // Euler angles, directly read
        match imu.euler_angles() {
            Ok(val) => {
                euler_angles = val;
                println!("IMU angles: {:?}", euler_angles);
                std::thread::sleep(std::time::Duration::from_millis(500));
            }
            Err(e) => {
                eprintln!("{:?}", e);
            }
        }
    }
}
