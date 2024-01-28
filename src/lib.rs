#![doc(html_root_url = "https://docs.rs/bno055/0.3.3")]
#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::bad_bit_mask)]

//! Bosch Sensortec BNO055 9-axis IMU sensor driver.
//! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

#[cfg(not(feature = "defmt-03"))]
use bitflags::bitflags;
#[cfg(feature = "defmt-03")]
use defmt::bitflags;

use byteorder::{ByteOrder, LittleEndian};
pub use mint;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

mod acc_config;
mod regs;
#[cfg(feature = "std")]
mod std;

pub use acc_config::{AccBandwidth, AccConfig, AccGRange, AccOperationMode};
pub use regs::BNO055_ID;

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),

    /// Invalid (not applicable) device mode.
    InvalidMode,

    /// Accelerometer configuration error
    AccConfig(acc_config::Error),
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055<I> {
    i2c: I,
    pub mode: BNO055OperationMode,
    use_default_addr: bool,
}

impl<I, E> Bno055<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I) -> Self {
        Bno055 {
            i2c,
            mode: BNO055OperationMode::CONFIG_MODE,
            use_default_addr: true,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// Enables use of alternative I2C address `regs::BNO055_ALTERNATE_ADDR`.
    pub fn with_alternative_address(mut self) -> Self {
        self.use_default_addr = false;

        self
    }

    /// Initializes the BNO055 device.
    ///
    /// Side-effects:
    /// - Software reset of BNO055
    /// - Sets BNO055 to `CONFIG` mode
    /// - Sets BNO055's power mode to `NORMAL`
    /// - Clears `SYS_TRIGGER` register
    ///
    /// # Usage Example
    ///
    /// ```rust
    /// // use your_chip_hal::{I2c, Delay}; // <- import your chip's I2c and Delay
    /// use bno055::Bno055;
    /// #
    /// # // All of this is needed for example to work:
    /// # use bno055::BNO055_ID;
    /// # use embedded_hal::delay::DelayNs;
    /// # use embedded_hal::i2c::{I2c as I2cTrait, Operation, Error, ErrorType, ErrorKind};
    /// # struct Delay {}
    /// # impl Delay { pub fn new() -> Self { Delay{ } }}
    /// # impl DelayNs for Delay {
    /// #    fn delay_ns(&mut self, ms: u32) {
    /// #        // no-op for example purposes
    /// #    }
    /// # }
    /// # struct I2c {}
    /// # impl I2c { pub fn new() -> Self { I2c { } }}
    /// # #[derive(Debug)]
    /// # struct DummyError {}
    /// # impl Error for DummyError { fn kind(&self) -> ErrorKind { ErrorKind::Other } }
    /// # impl ErrorType for I2c { type Error = DummyError; }
    /// # // 3 calls are made, 2 Writes and 1 Write/Read. We want to mock the 3rd call's read.
    /// # impl I2cTrait for I2c { fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> { match operations.get_mut(1) { Some(Operation::Read(read)) => { read[0] = BNO055_ID; }, _ => {} }; Ok(()) } }
    /// #
    /// # // Actual example:
    /// let mut delay = Delay::new(/* ... */);
    /// let mut i2c = I2c::new(/* ... */);
    /// let mut bno055 = Bno055::new(i2c);
    /// bno055.init(&mut delay)?;
    /// # Result::<(), bno055::Error<DummyError>>::Ok(())
    /// ```
    pub fn init(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let id = self.id()?;
        if id != regs::BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset(delay)?;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;
        self.set_power_mode(BNO055PowerMode::NORMAL)?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, 0x00)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    pub fn soft_reset(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            regs::BNO055_SYS_TRIGGER_RST_SYS_BIT,
        )
        .map_err(Error::I2c)?;

        // As per table 1.2
        delay.delay_ms(650);
        Ok(())
    }

    /// Sets the operating mode, see [BNO055OperationMode](enum.BNO055OperationMode.html).
    /// See section 3.3.
    pub fn set_mode(
        &mut self,
        mode: BNO055OperationMode,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        if self.mode != mode {
            self.set_page(BNO055RegisterPage::PAGE_0)?;

            self.mode = mode;

            self.write_u8(regs::BNO055_OPR_MODE, mode.bits())
                .map_err(Error::I2c)?;

            // Table 3-6 says 19ms to switch to CONFIG_MODE
            delay.delay_ms(19);
        }

        Ok(())
    }

    /// Sets the power mode, see [BNO055PowerMode](enum.BNO055PowerMode.html)
    /// See section 3.2
    pub fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(regs::BNO055_PWR_MODE, mode.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns BNO055's power mode.
    pub fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mode = self.read_u8(regs::BNO055_PWR_MODE).map_err(Error::I2c)?;

        Ok(BNO055PowerMode::from_bits_truncate(mode))
    }

    /// Enables/Disables usage of external 32k crystal.
    pub fn set_external_crystal(
        &mut self,
        ext: bool,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, if ext { 0x80 } else { 0x00 })
            .map_err(Error::I2c)?;

        self.set_mode(prev, delay)?;

        Ok(())
    }

    /// Configures axis remap of the device.
    pub fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let remap_value = (remap.x.bits() & 0b11)
            | ((remap.y.bits() & 0b11) << 2)
            | ((remap.z.bits() & 0b11) << 4);

        self.write_u8(regs::BNO055_AXIS_MAP_CONFIG, remap_value)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns axis remap of the device.
    pub fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_CONFIG)
            .map_err(Error::I2c)?;

        let remap = AxisRemap {
            x: BNO055AxisConfig::from_bits_truncate(value & 0b11),
            y: BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
            z: BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        };

        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(regs::BNO055_AXIS_MAP_SIGN, sign.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Return device's axes sign.
    pub fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_SIGN)
            .map_err(Error::I2c)?;

        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope of
    /// the BNO055 device.
    pub fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(regs::BNO055_ACC_ID, &mut buf)
            .map_err(Error::I2c)?;

        Ok(BNO055Revision {
            software: LittleEndian::read_u16(&buf[3..5]),
            bootloader: buf[5],
            accelerometer: buf[0],
            magnetometer: buf[1],
            gyroscope: buf[2],
        })
    }

    /// Returns device's system status.
    pub fn get_system_status(
        &mut self,
        do_selftest: bool,
        delay: &mut dyn DelayNs,
    ) -> Result<BNO055SystemStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

            let sys_trigger = self.read_u8(regs::BNO055_SYS_TRIGGER).map_err(Error::I2c)?;

            self.write_u8(regs::BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                .map_err(Error::I2c)?;

            // Wait for self-test result
            for _ in 0..4 {
                delay.delay_ms(255);
            }

            let result = self.read_u8(regs::BNO055_ST_RESULT).map_err(Error::I2c)?;

            self.set_mode(prev, delay)?; // Restore previous mode

            Some(BNO055SelfTestStatus::from_bits_truncate(result))
        } else {
            None
        };

        let status = self.read_u8(regs::BNO055_SYS_STATUS).map_err(Error::I2c)?;
        let error = self.read_u8(regs::BNO055_SYS_ERR).map_err(Error::I2c)?;

        Ok(BNO055SystemStatus {
            status: BNO055SystemStatusCode::from_bits_truncate(status),
            error: BNO055SystemErrorCode::from_bits_truncate(error),
            selftest,
        })
    }

    /// Gets a quaternion (`mint::Quaternion<f32>`) reading from the BNO055.
    /// Available only in sensor fusion modes.
    pub fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion mode to be able to produce quaternions
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let x = x as f32 * scale;
            let y = y as f32 * scale;
            let z = z as f32 * scale;
            let w = w as f32 * scale;

            let quat = mint::Quaternion {
                v: mint::Vector3 { x, y, z },
                s: w,
            };

            Ok(quat)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get Euler angles representation of heading in degrees.
    /// Euler angles is represented as (`roll`, `pitch`, `yaw/heading`).
    /// Available only in sensor fusion modes.
    pub fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion mode to be able to produce Euler angles
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 6] = [0; 6];

            self.read_bytes(regs::BNO055_EUL_HEADING_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let heading = LittleEndian::read_i16(&buf[0..2]) as f32;
            let roll = LittleEndian::read_i16(&buf[2..4]) as f32;
            let pitch = LittleEndian::read_i16(&buf[4..6]) as f32;

            let scale = 1f32 / 16f32; // 1 degree = 16 LSB

            let rot = mint::EulerAngles::from([roll * scale, pitch * scale, heading * scale]);

            Ok(rot)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get calibration status
    pub fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let status = self.read_u8(regs::BNO055_CALIB_STAT).map_err(Error::I2c)?;

        let sys = (status >> 6) & 0b11;
        let gyr = (status >> 4) & 0b11;
        let acc = (status >> 2) & 0b11;
        let mag = status & 0b11;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    /// Checks whether device is fully calibrated or not.
    pub fn is_fully_calibrated(&mut self) -> Result<bool, Error<E>> {
        let status = self.get_calibration_status()?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    /// Reads current calibration profile of the device.
    pub fn calibration_profile(
        &mut self,
        delay: &mut dyn DelayNs,
    ) -> Result<BNO055Calibration, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];

        self.read_bytes(regs::BNO055_ACC_OFFSET_X_LSB, &mut buf[..])
            .map_err(Error::I2c)?;

        let res = BNO055Calibration::from_buf(&buf);

        self.set_mode(prev_mode, delay)?;

        Ok(res)
    }

    /// Sets current calibration profile.
    pub fn set_calibration_profile(
        &mut self,
        calib: BNO055Calibration,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

        let buf_profile = calib.as_bytes();

        // Combine register address and profile into single buffer
        let buf_reg = [regs::BNO055_ACC_OFFSET_X_LSB; 1];
        let mut buf_with_reg = [0u8; 1 + BNO055_CALIB_SIZE];
        for (to, from) in buf_with_reg
            .iter_mut()
            .zip(buf_reg.iter().chain(buf_profile.iter()))
        {
            *to = *from
        }

        self.i2c
            .write(self.i2c_addr(), &buf_with_reg[..])
            .map_err(Error::I2c)?;

        self.set_mode(prev_mode, delay)?;

        Ok(())
    }

    /// Returns device's factory-programmed and constant chip ID.
    /// This ID is device model ID and not a BNO055's unique ID, whic is stored in different register.
    pub fn id(&mut self) -> Result<u8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;
        self.read_u8(regs::BNO055_CHIP_ID).map_err(Error::I2c)
    }

    /// Returns device's operation mode.
    pub fn get_mode(&mut self) -> Result<BNO055OperationMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mode = self.read_u8(regs::BNO055_OPR_MODE).map_err(Error::I2c)?;
        let mode = BNO055OperationMode::from_bits_truncate(mode);
        self.mode = mode;

        Ok(mode)
    }

    /// Checks whether the device is in Sensor Fusion mode or not.
    pub fn is_in_fusion_mode(&mut self) -> Result<bool, Error<E>> {
        Ok(self.mode.is_fusion_enabled())
    }

    pub fn get_acc_config(&mut self) -> Result<AccConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1)?;

        let bits = self.read_u8(regs::BNO055_ACC_CONFIG).map_err(Error::I2c)?;

        let acc_config = AccConfig::try_from_bits(bits).map_err(Error::AccConfig)?;

        Ok(acc_config)
    }

    pub fn set_acc_config(&mut self, acc_config: &AccConfig) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1)?;

        self.write_u8(regs::BNO055_ACC_CONFIG, acc_config.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets current register map page.
    fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(regs::BNO055_PAGE_ID, page.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Reads a vector of sensor data from the device.
    fn read_vec_raw(&mut self, reg: u8) -> Result<mint::Vector3<i16>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(reg, &mut buf).map_err(Error::I2c)?;

        let x = LittleEndian::read_i16(&buf[0..2]);
        let y = LittleEndian::read_i16(&buf[2..4]);
        let z = LittleEndian::read_i16(&buf[4..6]);

        Ok(mint::Vector3::from([x, y, z]))
    }

    /// Applies the given scaling to the vector of sensor data from the device.
    fn scale_vec(raw: mint::Vector3<i16>, scaling: f32) -> mint::Vector3<f32> {
        mint::Vector3::from([
            raw.x as f32 * scaling,
            raw.y as f32 * scaling,
            raw.z as f32 * scaling,
        ])
    }

    /// Returns linear acceleration vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn linear_acceleration_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_LIA_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns linear acceleration vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let linear_acceleration = self.linear_acceleration_fixed()?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(linear_acceleration, scaling))
    }

    /// Returns gravity vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn gravity_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_GRV_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns gravity vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let gravity = self.gravity_fixed()?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(gravity, scaling))
    }

    /// Returns current accelerometer data in cm/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub fn accel_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_accel_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current accelerometer data in m/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let a = self.accel_data_fixed()?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(a, scaling))
    }

    /// Returns current gyroscope data in 1/16th deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub fn gyro_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_gyro_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_GYR_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current gyroscope data in deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let g = self.gyro_data_fixed()?;
        let scaling = 1f32 / 16f32; // 1 deg/s = 16 lsb
        Ok(Self::scale_vec(g, scaling))
    }

    /// Returns current magnetometer data in 1/16th uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub fn mag_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_mag_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_MAG_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current magnetometer data in uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let m = self.mag_data_fixed()?;
        let scaling = 1f32 / 16f32; // 1 uT = 16 lsb
        Ok(Self::scale_vec(m, scaling))
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    pub fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Read temperature signed byte
        let temp = self.read_u8(regs::BNO055_TEMP).map_err(Error::I2c)? as i8;
        Ok(temp)
    }

    #[inline(always)]
    fn i2c_addr(&self) -> u8 {
        if !self.use_default_addr {
            regs::BNO055_ALTERNATE_ADDR
        } else {
            regs::BNO055_DEFAULT_ADDR
        }
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self.i2c.write_read(self.i2c_addr(), &[reg], &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(), &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(), &[reg, value])?;

        Ok(())
    }
}

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055AxisConfig: u8 {
        const AXIS_AS_X = 0b00;
        const AXIS_AS_Y = 0b01;
        const AXIS_AS_Z = 0b10;
    }
}

#[allow(clippy::misnamed_getters)]
impl AxisRemap {
    pub fn x(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn y(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn z(&self) -> BNO055AxisConfig {
        self.z
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisRemap {
    x: BNO055AxisConfig,
    y: BNO055AxisConfig,
    z: BNO055AxisConfig,
}

#[derive(Debug)]
pub struct AxisRemapBuilder {
    remap: AxisRemap,
}

impl AxisRemap {
    pub fn builder() -> AxisRemapBuilder {
        AxisRemapBuilder {
            remap: AxisRemap {
                x: BNO055AxisConfig::AXIS_AS_X,
                y: BNO055AxisConfig::AXIS_AS_Y,
                z: BNO055AxisConfig::AXIS_AS_Z,
            },
        }
    }
}

impl AxisRemapBuilder {
    pub fn swap_x_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_x = self.remap.x;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_x,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_x,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_x,

            _ => (),
        }

        self.remap.x = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_y_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_y = self.remap.y;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_y,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_y,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_y,

            _ => (),
        }

        self.remap.y = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_z_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_z = self.remap.z;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_z,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_z,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_z,

            _ => (),
        }

        self.remap.z = to;

        AxisRemapBuilder { remap: self.remap }
    }

    fn is_invalid(&self) -> bool {
        // Each axis must be swapped only once,
        // For example, one cannot remap X to Y and Z to Y at the same time, or similar.
        // See datasheet, section 3.4.
        self.remap.x == self.remap.y || self.remap.y == self.remap.z || self.remap.z == self.remap.x
    }

    #[allow(clippy::result_unit_err)]
    pub fn build(self) -> Result<AxisRemap, ()> {
        if self.is_invalid() {
            Err(())
        } else {
            Ok(self.remap)
        }
    }
}

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055AxisSign: u8 {
        const X_NEGATIVE = 0b100;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b001;
    }
}

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SystemStatusCode: u8 {
        const SYSTEM_IDLE = 0;
        const SYSTEM_ERROR = 1;
        const INIT_PERIPHERALS = 2;
        const SYSTEM_INIT = 3;
        const EXECUTING = 4;
        const RUNNING = 5;
        const RUNNING_WITHOUT_FUSION = 6;
    }
}

bitflags! {
    /// Possible BNO055 errors.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SystemErrorCode: u8 {
        const NONE = 0;
        const PERIPHERAL_INIT = 1;
        const SYSTEM_INIT = 2;
        const SELF_TEST = 3;
        const REGISTER_MAP_VALUE = 4;
        const REGISTER_MAP_ADDRESS = 5;
        const REGISTER_MAP_WRITE = 6;
        const LOW_POWER_MODE_NOT_AVAIL = 7;
        const ACCEL_POWER_MODE_NOT_AVAIL = 8;
        const FUSION_ALGO_CONFIG = 9;
        const SENSOR_CONFIG = 10;
    }
}

bitflags! {
    /// BNO055 self-test status bit flags.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SelfTestStatus: u8 {
        const ACC_OK = 0b0001;
        const MAG_OK = 0b0010;
        const GYR_OK = 0b0100;
        const SYS_OK = 0b1000;
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SystemStatus {
    status: BNO055SystemStatusCode,
    selftest: Option<BNO055SelfTestStatus>,
    error: BNO055SystemErrorCode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055Revision {
    pub software: u16,
    pub bootloader: u8,
    pub accelerometer: u8,
    pub magnetometer: u8,
    pub gyroscope: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(C)]
pub struct BNO055Calibration {
    pub acc_offset_x_lsb: u8,
    pub acc_offset_x_msb: u8,
    pub acc_offset_y_lsb: u8,
    pub acc_offset_y_msb: u8,
    pub acc_offset_z_lsb: u8,
    pub acc_offset_z_msb: u8,

    pub mag_offset_x_lsb: u8,
    pub mag_offset_x_msb: u8,
    pub mag_offset_y_lsb: u8,
    pub mag_offset_y_msb: u8,
    pub mag_offset_z_lsb: u8,
    pub mag_offset_z_msb: u8,

    pub gyr_offset_x_lsb: u8,
    pub gyr_offset_x_msb: u8,
    pub gyr_offset_y_lsb: u8,
    pub gyr_offset_y_msb: u8,
    pub gyr_offset_z_lsb: u8,
    pub gyr_offset_z_msb: u8,

    pub acc_radius_lsb: u8,
    pub acc_radius_msb: u8,
    pub mag_radius_lsb: u8,
    pub mag_radius_msb: u8,
}

/// BNO055's calibration profile size.
pub const BNO055_CALIB_SIZE: usize = core::mem::size_of::<BNO055Calibration>();

impl BNO055Calibration {
    pub fn from_buf(buf: &[u8; BNO055_CALIB_SIZE]) -> BNO055Calibration {
        unsafe { core::ptr::read(buf.as_ptr() as *const _) }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const _) as *const u8,
                ::core::mem::size_of::<BNO055Calibration>(),
            )
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

bitflags! {
    /// Possible BNO055 register map pages.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055RegisterPage: u8 {
        const PAGE_0 = 0;
        const PAGE_1 = 1;
    }
}

bitflags! {
    /// Possible BNO055 power modes.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055PowerMode: u8 {
        const NORMAL = 0b00;
        const LOW_POWER = 0b01;
        const SUSPEND = 0b10;
    }
}

bitflags! {
    /// Possible BNO055 operation modes.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055OperationMode: u8 {
        const CONFIG_MODE = 0b0000;
        const ACC_ONLY = 0b0001;
        const MAG_ONLY = 0b0010;
        const GYRO_ONLY = 0b0011;
        const ACC_MAG = 0b0100;
        const ACC_GYRO = 0b0101;
        const MAG_GYRO = 0b0110;
        const AMG = 0b0111;
        const IMU = 0b1000;
        const COMPASS = 0b1001;
        const M4G = 0b1010;
        const NDOF_FMC_OFF = 0b1011;
        const NDOF = 0b1100;
    }
}

impl BNO055OperationMode {
    fn is_fusion_enabled(&self) -> bool {
        matches!(
            *self,
            Self::IMU | Self::COMPASS | Self::M4G | Self::NDOF_FMC_OFF | Self::NDOF,
        )
    }

    fn is_accel_enabled(&self) -> bool {
        matches!(
            *self,
            Self::ACC_ONLY
                | Self::ACC_MAG
                | Self::ACC_GYRO
                | Self::AMG
                | Self::IMU
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    fn is_gyro_enabled(&self) -> bool {
        matches!(
            *self,
            Self::GYRO_ONLY
                | Self::ACC_GYRO
                | Self::MAG_GYRO
                | Self::AMG
                | Self::IMU
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    fn is_mag_enabled(&self) -> bool {
        matches!(
            *self,
            Self::MAG_ONLY
                | Self::ACC_MAG
                | Self::MAG_GYRO
                | Self::AMG
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }
}
