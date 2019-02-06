## Bosch Sensortec BNO055 embedded-hal driver

### What is this?

This is a [embedded-hal]() driver for Bosch's inertial measurement unit (IMU) BNO055.

### Usage

1. Add dependency to `Cargo.toml`:

    ```bash
    cargo add bno055
    ```
    
2. Instantiate and init the device:
    ```rust
    use bno055;

    pub fn init(i2c: I2c, delay: Delay) -> Result<bno055::Bno055, Error> {
       let imu = bno055::Bno055::new(i2c, delay);
    
       imu.init()?;
    
       // Enable 9-degrees-of-freedom sensor fusion mode with fast magnetometer calibration
       imu.set_mode(bno055::BNO055OperationMode::NDOF)?;
    
       Ok(imu)
    }
    ```

3. Read orientation data: quaternion or euler angles (roll, pitch, yaw/heading)
    ```rust
    let quat = imu.quaternion()?;
    // or:
    let euler = imu.euler_angles()?;
    ```
