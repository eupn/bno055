#![no_std]

///! Bosch Sensortec BNO055 9-axis IMU sensor driver.
///! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};

use bitflags::bitflags;
use byteorder::{ByteOrder, LittleEndian};
pub use nalgebra::{Quaternion, Rotation3};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),

    /// Invalid (not applicable) device mode.
    InvalidMode,
}

pub struct Bno055<I, D> {
    i2c: I,
    delay: D,
    pub mode: BNO055OperationMode,
}

impl<I, D, E> Bno055<I, D>
where
    I: WriteRead<Error = E> + Write<Error = E>,
    D: DelayMs<u8>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or write before `init()` call.
    pub fn new(i2c: I, delay: D) -> Self {
        let bno = Bno055 {
            i2c,
            delay,
            mode: BNO055OperationMode::CONFIG_MODE,
        };

        bno
    }

    /// Initializes the BNO055 device.
    ///
    /// Side-effects:
    /// - Software reset of BNO055
    /// - Sets BNO055 to CONFIG mode
    /// - Sets BNO055's power mode to NORMAL
    /// - Clears SYS_TRIGGER register
    pub fn init(&mut self) -> Result<(), Error<E>> {
        let id = self.id()?;
        if id != BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset()?;
        self.set_mode(BNO055OperationMode::CONFIG_MODE)?;
        self.set_power_mode(BNO055PowerMode::NORMAL)?;
        self.write_u8(BNO055_SYS_TRIGGER, 0x00)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_u8(BNO055_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS_BIT)
            .map_err(Error::I2c)
    }

    /// Sets the operating mode, see [BNO055OperationMode](enum.BNO055OperationMode.html).
    /// See section 3.3.
    pub fn set_mode(&mut self, mode: BNO055OperationMode) -> Result<(), Error<E>> {
        if self.mode != mode {
            self.mode = mode;

            self.write_u8(BNO055_OPR_MODE, mode.bits())
                .map_err(Error::I2c)?;

            // Table 3-6 says 19ms to switch to CONFIG_MODE
            self.delay.delay_ms(19);
        }

        Ok(())
    }

    /// Sets the register page.
    /// See section 4.2.
    pub fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(BNO055_PAGE_ID, page as u8)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets the power mode, see [BNO055PowerMode](enum.BNO055PowerMode.html)
    /// See section 3.2
    pub fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.write_u8(BNO055_PWR_MODE, mode.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns BNO055's power mode.
    pub fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        let mode = self.read_u8(BNO055_PWR_MODE).map_err(Error::I2c)?;

        Ok(BNO055PowerMode::from_bits_truncate(mode))
    }

    /// Enables/Disables usage of external 32k crystal.
    pub fn set_external_crystal(&mut self, ext: bool) -> Result<(), Error<E>> {
        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE)?;
        self.write_u8(BNO055_SYS_TRIGGER, if ext { 0x80 } else { 0x00 })
            .map_err(Error::I2c)?;

        self.set_mode(prev)?;

        Ok(())
    }

    /// Configures axis remap of the device.
    pub fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        let remap_value = ((remap.x.bits() & 0b11) << 0)
            | ((remap.y.bits() & 0b11) << 2)
            | ((remap.z.bits() & 0b11) << 4);

        self.write_u8(BNO055_AXIS_MAP_CONFIG, remap_value)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns axis remap of the device.
    pub fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        let value = self.read_u8(BNO055_AXIS_MAP_CONFIG).map_err(Error::I2c)?;

        let remap = AxisRemap {
            x: BNO055AxisConfig::from_bits_truncate((value >> 0) & 0b11),
            y: BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
            z: BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        };

        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.write_u8(BNO055_AXIS_MAP_SIGN, sign.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Return device's axes sign.
    pub fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        let value = self.read_u8(BNO055_AXIS_MAP_SIGN).map_err(Error::I2c)?;

        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope of
    /// the BNO055 device.
    pub fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(BNO055_ACC_ID, &mut buf)
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
    pub fn get_system_status(&mut self, do_selftest: bool) -> Result<BNO055SystemStatus, Error<E>> {
        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::CONFIG_MODE)?;

            let sys_trigger = self.read_u8(BNO055_SYS_TRIGGER).map_err(Error::I2c)?;

            self.write_u8(BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                .map_err(Error::I2c)?;

            // Wait for self-test result
            for _ in 0..4 {
                self.delay.delay_ms(255);
            }

            let result = self.read_u8(BNO055_ST_RESULT).map_err(Error::I2c)?;

            self.set_mode(prev)?; // Restore previous mode

            Some(BNO055SelfTestStatus::from_bits_truncate(result))
        } else {
            None
        };

        let status = self.read_u8(BNO055_SYS_STATUS).map_err(Error::I2c)?;
        let error = self.read_u8(BNO055_SYS_ERR).map_err(Error::I2c)?;

        Ok(BNO055SystemStatus {
            status: BNO055SystemStatusCode::from_bits_truncate(status),
            error: BNO055SystemErrorCode::from_bits_truncate(error),
            selftest,
        })
    }

    /// Gets a quaternion (`nalgebra::Quaternion<f32>`) reading from the BNO055.
    /// Must be in a sensor fusion (IMU) operating mode.
    pub fn quaternion(&mut self) -> Result<Quaternion<f32>, Error<E>> {
        // Device should be in fusion (IMU) mode to be able to produce quaternions
        if self.is_in_fusion_mode()? {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(BNO055_QUA_DATA_W_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let quat = Quaternion::new(
                w as f32 * scale,
                x as f32 * scale,
                y as f32 * scale,
                z as f32 * scale,
            );

            Ok(quat)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get Euler angles representation of heading in degrees.
    /// Euler angles is represented as (`roll`, `pitch`, `yaw/heading`).
    pub fn euler_angles(&mut self) -> Result<Rotation3<f32>, Error<E>> {
        // Device should be in fusion mode to be able to produce quaternions
        if self.is_in_fusion_mode()? {
            let mut buf: [u8; 6] = [0; 6];

            self.read_bytes(BNO055_EUL_HEADING_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let heading = LittleEndian::read_i16(&buf[0..2]) as f32;
            let roll = LittleEndian::read_i16(&buf[2..4]) as f32;
            let pitch = LittleEndian::read_i16(&buf[4..6]) as f32;

            let scale = 1f32 / 16f32; // 1 degree = 16 LSB

            let rot = Rotation3::from_euler_angles(roll * scale, pitch * scale, heading * scale);

            Ok(rot)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get calibration status
    pub fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        let status = self.read_u8(BNO055_CALIB_STAT).map_err(Error::I2c)?;

        let sys = (status >> 6) & 0b11;
        let gyr = (status >> 4) & 0b11;
        let acc = (status >> 2) & 0b11;
        let mag = (status >> 0) & 0b11;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    pub fn id(&mut self) -> Result<u8, Error<E>> {
        self.read_u8(BNO055_CHIP_ID).map_err(Error::I2c)
    }

    /// Returns device's operation mode.
    pub fn get_mode(&mut self) -> Result<BNO055OperationMode, Error<E>> {
        let mode = self.read_u8(BNO055_OPR_MODE).map_err(Error::I2c)?;
        let mode = BNO055OperationMode::from_bits_truncate(mode);
        self.mode = mode;

        Ok(mode)
    }

    /// Checks whether the device is in Sensor Fusion mode or not.
    pub fn is_in_fusion_mode(&mut self) -> Result<bool, Error<E>> {
        let is_in_fusion = match self.mode {
            BNO055OperationMode::IMU
            | BNO055OperationMode::COMPASS
            | BNO055OperationMode::M4G
            | BNO055OperationMode::NDOF_FMC_OFF
            | BNO055OperationMode::NDOF => true,

            _ => false,
        };

        Ok(is_in_fusion)
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self.i2c.write_read(BNO055_DEFAULT_ADDR, &[reg], &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(BNO055_DEFAULT_ADDR, &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(BNO055_DEFAULT_ADDR, &[reg, value])?;

        Ok(())
    }
}

// --- Regs definition ---

pub const BNO055_DEFAULT_ADDR: u8 = 0x28;
pub const BNO055_ALTERNATE_ADDR: u8 = 0x29;
pub const BNO055_ID: u8 = 0xA0;

pub const BNO055_PAGE_ID: u8 = 0x07;

pub const BNO055_CHIP_ID: u8 = 0x00;
pub const BNO055_ACC_ID: u8 = 0x01;
pub const BNO055_MAG_ID: u8 = 0x02;
pub const BNO055_GYR_ID: u8 = 0x03;
pub const BNO055_SW_REV_ID_LSB: u8 = 0x04;
pub const BNO055_SW_REV_ID_MSB: u8 = 0x05;
pub const BNO055_BL_REV_ID: u8 = 0x06;

pub const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
pub const BNO055_ACC_DATA_X_MSB: u8 = 0x09;
pub const BNO055_ACC_DATA_Y_LSB: u8 = 0x0A;
pub const BNO055_ACC_DATA_Y_MSB: u8 = 0x0B;
pub const BNO055_ACC_DATA_Z_LSB: u8 = 0x0C;
pub const BNO055_ACC_DATA_Z_MSB: u8 = 0x0D;

pub const BNO055_MAG_DATA_X_LSB: u8 = 0x0E;
pub const BNO055_MAG_DATA_X_MSB: u8 = 0x0F;
pub const BNO055_MAG_DATA_Y_LSB: u8 = 0x10;
pub const BNO055_MAG_DATA_Y_MSB: u8 = 0x11;
pub const BNO055_MAG_DATA_Z_LSB: u8 = 0x12;
pub const BNO055_MAG_DATA_Z_MSB: u8 = 0x13;

pub const BNO055_GYR_DATA_X_LSB: u8 = 0x14;
pub const BNO055_GYR_DATA_X_MSB: u8 = 0x15;
pub const BNO055_GYR_DATA_Y_LSB: u8 = 0x16;
pub const BNO055_GYR_DATA_Y_MSB: u8 = 0x17;
pub const BNO055_GYR_DATA_Z_LSB: u8 = 0x18;
pub const BNO055_GYR_DATA_Z_MSB: u8 = 0x19;

pub const BNO055_EUL_HEADING_LSB: u8 = 0x1A;
pub const BNO055_EUL_HEADING_MSB: u8 = 0x1B;
pub const BNO055_EUL_ROLL_LSB: u8 = 0x1C;
pub const BNO055_EUL_ROLL_MSB: u8 = 0x1D;
pub const BNO055_EUL_PITCH_LSB: u8 = 0x1E;
pub const BNO055_EUL_PITCH_MSB: u8 = 0x1F;

/// Quaternion data
pub const BNO055_QUA_DATA_W_LSB: u8 = 0x20;
pub const BNO055_QUA_DATA_W_MSB: u8 = 0x21;
pub const BNO055_QUA_DATA_X_LSB: u8 = 0x22;
pub const BNO055_QUA_DATA_X_MSB: u8 = 0x23;
pub const BNO055_QUA_DATA_Y_LSB: u8 = 0x24;
pub const BNO055_QUA_DATA_Y_MSB: u8 = 0x25;
pub const BNO055_QUA_DATA_Z_LSB: u8 = 0x26;
pub const BNO055_QUA_DATA_Z_MSB: u8 = 0x27;

/// Linear acceleration data
pub const BNO055_LIA_DATA_X_LSB: u8 = 0x28;
pub const BNO055_LIA_DATA_X_MSB: u8 = 0x29;
pub const BNO055_LIA_DATA_Y_LSB: u8 = 0x2A;
pub const BNO055_LIA_DATA_Y_MSB: u8 = 0x2B;
pub const BNO055_LIA_DATA_Z_LSB: u8 = 0x2C;
pub const BNO055_LIA_DATA_Z_MSB: u8 = 0x2D;

/// Gravity vector data
pub const BNO055_GRV_DATA_X_LSB: u8 = 0x2E;
pub const BNO055_GRV_DATA_X_MSB: u8 = 0x2F;
pub const BNO055_GRV_DATA_Y_LSB: u8 = 0x30;
pub const BNO055_GRV_DATA_Y_MSB: u8 = 0x31;
pub const BNO055_GRV_DATA_Z_LSB: u8 = 0x32;
pub const BNO055_GRV_DATA_Z_MSB: u8 = 0x33;

/// Temperature data
pub const BNO055_TEMP: u8 = 0x34;

/// Calibration Status
pub const BNO055_CALIB_STAT: u8 = 0x35;

pub const BNO055_ST_RESULT: u8 = 0x36;
pub const BNO055_INT_STA: u8 = 0x37;
pub const BNO055_SYS_CLK_STATUS: u8 = 0x38;
pub const BNO055_SYS_STATUS: u8 = 0x39;
pub const BNO055_SYS_ERR: u8 = 0x3A;
pub const BNO055_UNIT_SEL: u8 = 0x3B;
pub const BNO055_OPR_MODE: u8 = 0x3D;
pub const BNO055_PWR_MODE: u8 = 0x3E;

pub const BNO055_SYS_TRIGGER: u8 = 0x3F;
pub const BNO055_SYS_TRIGGER_RST_SYS_BIT: u8 = 0x20; // Reset command
pub const BNO055_SYS_TRIGGER_SELF_TEST_BIT: u8 = 0b000_0001; // Self-test command
pub const BNO055_TEMP_SOURCE: u8 = 0x40;
pub const BNO055_AXIS_MAP_CONFIG: u8 = 0x41;
pub const BNO055_AXIS_MAP_SIGN: u8 = 0x42;

pub const BNO055_ACC_OFFSET_X_LSB: u8 = 0x55;
pub const BNO055_ACC_OFFSET_X_MSB: u8 = 0x56;
pub const BNO055_ACC_OFFSET_Y_LSB: u8 = 0x57;
pub const BNO055_ACC_OFFSET_Y_MSB: u8 = 0x58;
pub const BNO055_ACC_OFFSET_Z_LSB: u8 = 0x59;
pub const BNO055_ACC_OFFSET_Z_MSB: u8 = 0x5A;

pub const BNO055_MAG_OFFSET_X_LSB: u8 = 0x5B;
pub const BNO055_MAG_OFFSET_X_MSB: u8 = 0x5C;
pub const BNO055_MAG_OFFSET_Y_LSB: u8 = 0x5D;
pub const BNO055_MAG_OFFSET_Y_MSB: u8 = 0x5E;
pub const BNO055_MAG_OFFSET_Z_LSB: u8 = 0x5F;
pub const BNO055_MAG_OFFSET_Z_MSB: u8 = 0x60;

pub const BNO055_GYR_OFFSET_X_LSB: u8 = 0x61;
pub const BNO055_GYR_OFFSET_X_MSB: u8 = 0x62;
pub const BNO055_GYR_OFFSET_Y_LSB: u8 = 0x63;
pub const BNO055_GYR_OFFSET_Y_MSB: u8 = 0x64;
pub const BNO055_GYR_OFFSET_Z_LSB: u8 = 0x65;
pub const BNO055_GYR_OFFSET_Z_MSB: u8 = 0x66;

pub const BNO055_ACC_RADIUS_LSB: u8 = 0x67;
pub const BNO055_ACC_RADIUS_MSB: u8 = 0x68;
pub const BNO055_MAG_RADIUS_LSB: u8 = 0x69;
pub const BNO055_MAG_RADIUS_MSB: u8 = 0x6A;

bitflags! {
    pub struct BNO055AxisConfig: u8 {
        const AXIS_AS_X = 0b00;
        const AXIS_AS_Y = 0b01;
        const AXIS_AS_Z = 0b10;
    }
}

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

#[derive(Debug)]
pub struct AxisRemap {
    x: BNO055AxisConfig,
    y: BNO055AxisConfig,
    z: BNO055AxisConfig,
}

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

    pub fn build(self) -> Result<AxisRemap, ()> {
        if self.is_invalid() {
            Err(())
        } else {
            Ok(self.remap)
        }
    }
}

bitflags! {
    pub struct BNO055AxisSign: u8 {
        const X_NEGATIVE = 0b001;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b100;
    }
}

bitflags! {
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
    pub struct BNO055SelfTestStatus: u8 {
        const ACC_OK = 0b0001;
        const MAG_OK = 0b0010;
        const GYR_OK = 0b0100;
        const SYS_OK = 0b1000;
    }
}

#[derive(Debug)]
pub struct BNO055SystemStatus {
    status: BNO055SystemStatusCode,
    selftest: Option<BNO055SelfTestStatus>,
    error: BNO055SystemErrorCode,
}

#[derive(Debug)]
pub struct BNO055Revision {
    pub software: u16,
    pub bootloader: u8,
    pub accelerometer: u8,
    pub magnetometer: u8,
    pub gyroscope: u8,
}

#[derive(Debug)]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum BNO055RegisterPage {
    Page0 = 0,
    Page1 = 1,
}

bitflags! {
    /// Possible BNO055 power modes.
    pub struct BNO055PowerMode: u8 {
        const NORMAL = 0b00;
        const LOW_POWER = 0b01;
        const SUSPEND = 0b10;
    }
}

bitflags! {
    /// Possible BNO055 operation modes.
    pub struct BNO055OperationMode: u8 {
        const CONFIG_MODE = 0b0000;
        const ACC_ONLY = 0b0001;
        const MAG_ONLY = 0b0010;
        const GYRO_ONLU = 0b0011;
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
