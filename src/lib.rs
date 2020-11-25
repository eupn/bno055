#![no_std]

///! Bosch Sensortec BNO055 9-axis IMU sensor driver.
///! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};

use bitflags::bitflags;
use byteorder::{ByteOrder, LittleEndian};
pub use mint;

mod regs;

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

pub struct Bno055<I> {
    i2c: I,
    pub mode: BNO055OperationMode,
    use_default_addr: bool,
}

macro_rules! set_u8_from {
    ($bno055 : expr, $page : expr, $reg : expr, $x : expr) => {{
        $bno055.set_page($page)?;
        $bno055.write_u8($reg, $x.into())
            .map_err(Error::I2c)?;
        Ok(())
    }};
}

macro_rules! set_config_from {
    ($bno055 : expr, $page : expr, $reg : expr, $x : expr, $delay : expr) => {{
        let prev = $bno055.mode;
        $bno055.set_mode(BNO055OperationMode::CONFIG_MODE, $delay)?;
        let res = set_u8_from!($bno055, $page, $reg, $x);
        $bno055.set_mode(prev, $delay)?;
        res
    }};
}

macro_rules! read_u8_into {
    ($bno055 : expr, $page : expr, $reg : expr) => {{
        $bno055.set_page($page)?;
        let regval = $bno055.read_u8($reg)
            .map_err(Error::I2c)?;
        Ok(regval.into())
    }};
}

macro_rules! write_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flags : expr) => {{
        $bno055.set_page($page)?;
        $bno055.write_u8($reg, $flags.bits())
            .map_err(Error::I2c)?;
        Ok(())
    }};
}

macro_rules! write_config_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flags : expr, $delay : expr) => {{
        let prev = $bno055.mode;
        $bno055.set_mode(BNO055OperationMode::CONFIG_MODE, $delay)?;
        let res = write_flags!($bno055, $page, $reg, $flags);
        $bno055.set_mode(prev, $delay)?;
        res
    }};
}

macro_rules! read_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flag_type : ty) => {{
        $bno055.set_page($page)?;
        let flags = $bno055.read_u8($reg).map_err(Error::I2c)?;
        let flags = <$flag_type>::from_bits_truncate(flags);
        Ok(flags)
    }};
}

impl<I, E> Bno055<I>
where
    I: WriteRead<Error = E> + Write<Error = E>,
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

    /// Enables use of alternative I2C address `regs::BNO055_ALTERNATE_ADDR`.
    pub fn with_alternative_address(mut self) -> Self {
        self.use_default_addr = false;

        self
    }

    /// Initializes the BNO055 device.
    ///
    /// Side-effects:
    /// - Software reset of BNO055
    /// - Sets BNO055 to CONFIG mode
    /// - Sets BNO055's power mode to NORMAL
    /// - Clears SYS_TRIGGER register
    pub fn init(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
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
    pub fn soft_reset(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
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
        delay: &mut dyn DelayMs<u16>,
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
        delay: &mut dyn DelayMs<u16>,
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
        delay: &mut dyn DelayMs<u16>,
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
    /// Must be in a sensor fusion (IMU) operating mode.
    pub fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion (IMU) mode to be able to produce quaternions
        if self.is_in_fusion_mode()? {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let quat = mint::Quaternion::from([
                w as f32 * scale,
                x as f32 * scale,
                y as f32 * scale,
                z as f32 * scale,
            ]);

            Ok(quat)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get Euler angles representation of heading in degrees.
    /// Euler angles is represented as (`roll`, `pitch`, `yaw/heading`).
    pub fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion mode to be able to produce quaternions
        if self.is_in_fusion_mode()? {
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
        delay: &mut dyn DelayMs<u16>,
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
        delay: &mut dyn DelayMs<u16>,
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

    /// Sets current register map page.
    fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(regs::BNO055_PAGE_ID, page.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Reads vector of sensor data from the device.
    fn read_vec(&mut self, reg: u8, scaling: f32) -> Result<mint::Vector3<f32>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(reg, &mut buf).map_err(Error::I2c)?;

        let x = LittleEndian::read_i16(&buf[0..2]) as f32;
        let y = LittleEndian::read_i16(&buf[2..4]) as f32;
        let z = LittleEndian::read_i16(&buf[4..6]) as f32;

        Ok(mint::Vector3::from([x * scaling, y * scaling, z * scaling]))
    }

    /// Returns linear acceleration vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        if self.is_in_fusion_mode()? {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
            self.read_vec(regs::BNO055_LIA_DATA_X_LSB, scaling)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns gravity vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        if self.is_in_fusion_mode()? {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
            self.read_vec(regs::BNO055_GRV_DATA_X_LSB, scaling)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current accelerometer data in m/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        match self.mode {
            BNO055OperationMode::ACC_ONLY
            | BNO055OperationMode::ACC_GYRO
            | BNO055OperationMode::ACC_MAG
            | BNO055OperationMode::AMG => {
                self.set_page(BNO055RegisterPage::PAGE_0)?;
                let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
                self.read_vec(regs::BNO055_ACC_DATA_X_LSB, scaling)
            }

            _ => Err(Error::InvalidMode),
        }
    }

    /// Returns current gyroscope data in deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        match self.mode {
            BNO055OperationMode::GYRO_ONLY
            | BNO055OperationMode::ACC_GYRO
            | BNO055OperationMode::MAG_GYRO
            | BNO055OperationMode::AMG => {
                self.set_page(BNO055RegisterPage::PAGE_0)?;
                let scaling = 1f32 / 16f32; // 1 deg/s = 16 lsb
                self.read_vec(regs::BNO055_GYR_DATA_X_LSB, scaling)
            }

            _ => Err(Error::InvalidMode),
        }
    }

    /// Returns current magnetometer data in uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        match self.mode {
            BNO055OperationMode::MAG_ONLY
            | BNO055OperationMode::ACC_MAG
            | BNO055OperationMode::MAG_GYRO
            | BNO055OperationMode::AMG => {
                self.set_page(BNO055RegisterPage::PAGE_0)?;
                let scaling = 1f32 / 16f32; // 1 uT = 16 lsb
                self.read_vec(regs::BNO055_MAG_DATA_X_LSB, scaling)
            }

            _ => Err(Error::InvalidMode),
        }
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    pub fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Read temperature signed byte
        let temp = self.read_u8(regs::BNO055_TEMP).map_err(Error::I2c)? as i8;
        Ok(temp)
    }

    /// Read which interrupts are currently triggered/active
    pub fn interrupts_triggered(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        read_flags!(self, BNO055RegisterPage::PAGE_0, regs::BNO055_INT_STA, BNO055Interrupt)
    }

    /// Resets the interrupts register and the INT pin
    pub fn clear_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            regs::BNO055_SYS_TRIGGER_RST_INT_BIT,
        )
        .map_err(Error::I2c)?;
        Ok(())
    }

    /// Sets which interrupts are enabled
    /// One of the only config options that dont need to be in config mode to write to
    pub fn set_interrupts_enabled(&mut self, interrupts: BNO055Interrupt) -> Result<(), Error<E>> {
        write_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_INT_EN, interrupts)
    }

    /// Returns currently enabled interrupts
    pub fn interrupts_enabled(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        read_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_INT_EN, BNO055Interrupt)
    }

    /// Sets interrupts mask
    /// Official doc: when mask=1, the interrupt will update INT_STA register and trigger a change in INT pin,
    /// When mask=0, only INT_STA register will be updated
    /// One of the only config options that dont need to be in config mode to write to
    pub fn set_interrupts_mask(&mut self, mask: BNO055Interrupt) -> Result<(), Error<E>> {
        write_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_INT_MSK, mask)
    }

    /// Returns the current interrupts mask
    pub fn interrupts_mask(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        read_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_INT_MSK, BNO055Interrupt)
    }
    
    /// Sets accelerometer interrupt settings
    pub fn set_acc_interrupt_settings(&mut self, settings: BNO055AccIntSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_INT_SETTING, settings, delay)
    }

    /// Returns current accelerometer interrupt settings
    pub fn acc_interrupt_settings(&mut self) -> Result<BNO055AccIntSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_INT_SETTING)
    }

    /// Sets accelerometer High-G interrupt duration setting
    pub fn set_acc_hg_duration(&mut self, dur: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_DURATION, dur, delay)
    }

    /// Returns current accelerometer High-G interrupt duration setting
    pub fn acc_hg_duration(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_DURATION)
    }

    /// Sets accelerometer High-G interrupt threshold setting
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn set_acc_hg_threshold(&mut self, mult: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_THRES, mult, delay)
    }

    /// Returns current accelerometer High-G interrupt threshold setting
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn acc_hg_threshold(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_THRES)
    }

    /// Sets accelerometer no/slow-motion interrupt threshold setting
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn set_acc_nm_threshold(&mut self, mult: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_THRES, mult, delay)
    }

    /// Returns current accelerometer no/slow-motion interrupt threshold setting
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn acc_nm_threshold(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_THRES)
    }

    /// Sets accelerometer no/slow-motion interrupt settings
    pub fn set_acc_nm_settings(&mut self, settings: BNO055AccNmSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_SET, settings, delay)
    }

    /// Returns current accelerometer no/slow-motion interrupt settings
    pub fn acc_nm_settings(&mut self) -> Result<BNO055AccNmSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_SET)
    }

    /// Sets gyroscope interrupt settings
    pub fn set_gyr_interrupt_settings(&mut self, settings: BNO055GyrIntSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        write_config_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_INT_SETTING, settings, delay)
    }

    /// Returns the current gyroscope interrupt settings
    pub fn gyr_interrupt_settings(&mut self) -> Result<BNO055GyrIntSettings, Error<E>> {
        read_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_INT_SETTING, BNO055GyrIntSettings)
    }

    /// Sets gyroscope high-rate interrupt settings for x-axis
    pub fn set_gyr_hr_x_settings(&mut self, settings: BNO055GyrHrSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_X_SET, settings, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    pub fn gyr_hr_x_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_X_SET)
    }

    /// Sets gyroscope high-rate interrupt duration for x-axis
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_x(&mut self, duration: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_X, duration, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_x(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_X)
    }

    /// Sets gyroscope high-rate interrupt settings for y-axis
    pub fn set_gyr_hr_y_settings(&mut self, settings: BNO055GyrHrSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Y_SET, settings, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    pub fn gyr_hr_y_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Y_SET)
    }

    /// Sets gyroscope high-rate interrupt duration for y-axis
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_y(&mut self, duration: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Y, duration, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_y(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Y)
    }

    /// Sets gyroscope high-rate interrupt settings for z-axis
    pub fn set_gyr_hr_z_settings(&mut self, settings: BNO055GyrHrSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Z_SET, settings, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    pub fn gyr_hr_z_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Z_SET)
    }

    /// Sets gyroscope high-rate interrupt duration for z-axis
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_z(&mut self, duration: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z, duration, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_z(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z)
    }

    /// Sets gyroscope any-motion interrupt threshold
    /// Actual value is `mult` * base-unit based on gyroscope range set in GYR_CONFIG_0
    pub fn set_gyr_am_threshold(&mut self, mult: u8, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        let mut mult = mult;
        if mult > 0b01111111 {
            mult = 0b01111111;
        }
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z, mult & BIT_7_RESERVED_MASK, delay)
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    /// Actual value is `mult` * base-unit based on gyroscope range set in GYR_CONFIG_0
    pub fn gyr_am_threshold(&mut self) -> Result<u8, Error<E>> {
        let res: Result<u8, Error<E>> = read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z);
        res.map(|mult| BIT_7_RESERVED_MASK & mult)
    }

    /// Sets gyroscope any-motion interrupt settings
    pub fn set_gyr_am_settings(&mut self, settings: BNO055GyrAmSettings, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        set_config_from!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_AM_SET, settings, delay)
    }

    /// Returns current gyroscope any-motion interrupt settings
    pub fn gyr_am_settings(&mut self) -> Result<BNO055GyrAmSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_AM_SET)
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
        const X_NEGATIVE = 0b100;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b001;
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

#[derive(Debug)]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

bitflags! {
    /// Possible BNO055 register map pages.
    pub struct BNO055RegisterPage: u8 {
        const PAGE_0 = 0;
        const PAGE_1 = 1;
    }
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

bitflags! {
    /// BNO055 interrupt enable/mask flags.
    pub struct BNO055Interrupt: u8 {
        const ACC_NM = 0b10000000;
        const ACC_AM = 0b01000000;
        const ACC_HIGH_G = 0b00100000;
        const GYR_DRDY = 0b00010000;
        const GYR_HIGH_RATE = 0b00001000;
        const GYRO_AM = 0b00000100;
        const MAG_DRDY = 0b00000010;
        const ACC_BSX_DRDY = 0b00000001;
    }
}

bitflags! {
    /// BNO055 accelerometer interrupt settings
    pub struct BNO055AccIntSettingsFlags: u8 {
        const HG_Z_AXIS = 0b10000000;
        const HG_Y_AXIS = 0b01000000;
        const HG_X_AXIS = 0b00100000;
        const AMNM_Z_AXIS = 0b00010000;
        const AMNM_Y_AXIS = 0b00001000;
        const AMNM_X_AXIS = 0b00000100;
    }
}

const BIT_7_RESERVED_MASK: u8 = 0b01111111;

#[derive(Debug)]
pub struct BNO055AccIntSettings {
    pub flags: BNO055AccIntSettingsFlags,
    /// `am_dur` in [0, 3]
    pub am_dur: u8
}

impl From<u8> for BNO055AccIntSettings {
    fn from(regval: u8) -> Self {
        Self {
            flags: BNO055AccIntSettingsFlags::from_bits_truncate(regval),
            am_dur: regval & 3
        }
    }
}

impl Into<u8> for BNO055AccIntSettings {
    fn into(self) -> u8 {
        let mut dur = self.am_dur;
        if dur > 3 {
            dur = 3;
        }
        self.flags.bits() | dur
    }
}

#[derive(Debug)]
pub struct BNO055AccNmSettings {
    /// `dur` in [0, 0b111111]. Details of how actual duration is calculated in official doc
    pub dur: u8,
    /// true - no motion, false - slow motion 
    pub is_no_motion: bool,
}

impl From<u8> for BNO055AccNmSettings {
    fn from(regval: u8) -> Self {
        Self {
            dur: (regval & 0b01111110) >> 1,
            is_no_motion: match regval & 1 {
                0 => false,
                _ => true
            }
        }
    }
}

impl Into<u8> for BNO055AccNmSettings {
    fn into(self) -> u8 {
        let mut dur = self.dur;
        if dur > 0b111111 {
            dur = 0b111111;
        }
        let is_no_motion = match self.is_no_motion {
            true => 1,
            false => 0,
        };
        BIT_7_RESERVED_MASK & ((dur << 1) | is_no_motion)
    }
}

bitflags! {
    /// BNO055 gyroscope interrupt settings
    pub struct BNO055GyrIntSettings: u8 {
        const HR_FILT = 0b10000000;
        const AM_FILT = 0b01000000;
        const HR_Z_AXIS = 0b00100000;
        const HR_Y_AXIS = 0b00010000;
        const HR_X_AXIS = 0b00001000;
        const AM_Z_AXIS = 0b00000100;
        const AM_Y_AXIS = 0b00000010;
        const AM_X_AXIS = 0b00000001;
    }
}

/// Gyroscope High Rate interrupt settings
#[derive(Debug)]
pub struct BNO055GyrHrSettings {
    /// `hysteresis` in [0, 0b11]. Actual value is `hysteresis` * base-unit based on gyroscope range set in GYR_CONFIG
    pub hysteresis: u8,
    /// `threshold` in [0, 0b11111]. Actual value is `threshold` * base-unit based on gyroscope range set in GYR_CONFIG
    pub threshold: u8,
}

impl From<u8> for BNO055GyrHrSettings {
    fn from(regval: u8) -> Self {
        Self {
            hysteresis: (regval & 0b01100000) >> 5,
            threshold: regval & 0b00011111
        }
    }
}

impl Into<u8> for BNO055GyrHrSettings {
    fn into(self) -> u8 {
        let mut hysteresis = self.hysteresis;
        if hysteresis > 0b11 {
            hysteresis = 0b11;
        }
        let mut threshold = self.threshold;
        if threshold > 0b11111 {
            threshold = 0b11111;
        }
        BIT_7_RESERVED_MASK & (hysteresis << 5 | threshold)
    }
}

/// Gyroscope Any Motion interrupt settings
#[derive(Debug)]
pub struct BNO055GyrAmSettings {
    pub awake_duration: GyrAmSamplesAwake,
    /// `slope_samples` in [0, 0b11]. Actual value is (`slope_samples` + 1) * 4
    pub slope_samples: u8,
}

#[derive(Debug)]
pub enum GyrAmSamplesAwake {
    Samples8,
    Samples16,
    Samples32,
    Samples64,
}

impl From<u8> for GyrAmSamplesAwake {
    fn from(regval: u8) -> Self {
        match regval {
            0 => Self::Samples8,
            1 => Self::Samples16,
            2 => Self::Samples32,
            3 => Self::Samples64,
            _ => Self::Samples8, // TODO: handle error case?
        }
    }
}

impl Into<u8> for GyrAmSamplesAwake {
    fn into(self) -> u8 {
        match self {
            Self::Samples8 => 0,
            Self::Samples16 => 1,
            Self::Samples32 => 2,
            Self::Samples64 => 3,
        }
    }
}

impl From<u8> for BNO055GyrAmSettings {
    fn from(regval: u8) -> Self {
        Self {
            awake_duration: ((regval >> 2) & 3).into(),
            slope_samples: regval & 3
        }
    }
}

impl Into<u8> for BNO055GyrAmSettings {
    fn into(self) -> u8 {
        let mut slope_samples = self.slope_samples;
        if slope_samples > 0b11 {
            slope_samples = 0b11;
        }
        let awake_duration: u8 = self.awake_duration.into();
        0b00001111 & (awake_duration | slope_samples)
    }
}