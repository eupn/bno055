## Bosch Sensortec BNO055 embedded-hal driver

[![Build Status](https://travis-ci.org/copterust/bno055.svg?branch=master)](https://travis-ci.org/eupn/bno055)
![](https://img.shields.io/crates/v/bno055.svg?style=flat)
![](https://img.shields.io/crates/d/bno055.svg?maxAge=3600)

## What is this?

This is a [embedded-hal](https://github.com/rust-embedded/embedded-hal) driver for Bosch's inertial measurement unit (IMU) BNO055.

It is device-agnostic and uses `I2c` and `Delay` embedded-hal traits for its operation.

Uses and re-exports [nalgebra](https://www.nalgebra.org/)'s [Quaternion](http://toxiclibs.org/docs/core/toxi/geom/Quaternion.html) for quaternion reading and [Rotation3](https://www.nalgebra.org/rustdoc/nalgebra/geometry/type.Rotation3.html) for Euler angles.

## Usage

1. Add dependency to `Cargo.toml`:

    ```bash
    cargo add bno055
    ```
    
2. Instantiate and init the device:
    ```rust
    // ... declare and configure your I2c and Delay implementations ...
    
    // Init BNO055 IMU
    let imu = bno055::Bno055::new(i2c, delay);
    
    imu.init()?;
    
    // Enable 9-degrees-of-freedom sensor fusion mode with fast magnetometer calibration
    imu.set_mode(bno055::BNO055OperationMode::NDOF)?;
    
    Ok(imu)
    ```

3. Read orientation data: quaternion or euler angles (roll, pitch, yaw/heading)
    ```rust
    let quat = imu.quaternion()?;
    // or:
    let euler = imu.euler_angles()?;
    ```

## Status

What is done and tested and what is not yet:

- [x] Sensor initialization
- [x] Device mode setup
- [x] Device status readout
- [x] Calibration status readout
- [ ] Calibration data readout
- [ ] Calibration data setup
- [ ] Orientation data readout
    - [x] Quaternions
    - [x] Euler angles
    - [ ] Raw accelerometer data readout
    - [ ] Raw gyroscope data readout
    - [ ] Raw magnetometer data readout
