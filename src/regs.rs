#![allow(dead_code)]

pub(crate) const BNO055_DEFAULT_ADDR: u8 = 0x29;
pub(crate) const BNO055_ALTERNATE_ADDR: u8 = 0x28;
pub(crate) const BNO055_ID: u8 = 0xA0;

pub(crate) const BNO055_PAGE_ID: u8 = 0x07;

pub(crate) const BNO055_CHIP_ID: u8 = 0x00;
pub(crate) const BNO055_ACC_ID: u8 = 0x01;
pub(crate) const BNO055_MAG_ID: u8 = 0x02;
pub(crate) const BNO055_GYR_ID: u8 = 0x03;
pub(crate) const BNO055_SW_REV_ID_LSB: u8 = 0x04;
pub(crate) const BNO055_SW_REV_ID_MSB: u8 = 0x05;
pub(crate) const BNO055_BL_REV_ID: u8 = 0x06;

pub(crate) const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
pub(crate) const BNO055_ACC_DATA_X_MSB: u8 = 0x09;
pub(crate) const BNO055_ACC_DATA_Y_LSB: u8 = 0x0A;
pub(crate) const BNO055_ACC_DATA_Y_MSB: u8 = 0x0B;
pub(crate) const BNO055_ACC_DATA_Z_LSB: u8 = 0x0C;
pub(crate) const BNO055_ACC_DATA_Z_MSB: u8 = 0x0D;

pub(crate) const BNO055_MAG_DATA_X_LSB: u8 = 0x0E;
pub(crate) const BNO055_MAG_DATA_X_MSB: u8 = 0x0F;
pub(crate) const BNO055_MAG_DATA_Y_LSB: u8 = 0x10;
pub(crate) const BNO055_MAG_DATA_Y_MSB: u8 = 0x11;
pub(crate) const BNO055_MAG_DATA_Z_LSB: u8 = 0x12;
pub(crate) const BNO055_MAG_DATA_Z_MSB: u8 = 0x13;

pub(crate) const BNO055_GYR_DATA_X_LSB: u8 = 0x14;
pub(crate) const BNO055_GYR_DATA_X_MSB: u8 = 0x15;
pub(crate) const BNO055_GYR_DATA_Y_LSB: u8 = 0x16;
pub(crate) const BNO055_GYR_DATA_Y_MSB: u8 = 0x17;
pub(crate) const BNO055_GYR_DATA_Z_LSB: u8 = 0x18;
pub(crate) const BNO055_GYR_DATA_Z_MSB: u8 = 0x19;

pub(crate) const BNO055_EUL_HEADING_LSB: u8 = 0x1A;
pub(crate) const BNO055_EUL_HEADING_MSB: u8 = 0x1B;
pub(crate) const BNO055_EUL_ROLL_LSB: u8 = 0x1C;
pub(crate) const BNO055_EUL_ROLL_MSB: u8 = 0x1D;
pub(crate) const BNO055_EUL_PITCH_LSB: u8 = 0x1E;
pub(crate) const BNO055_EUL_PITCH_MSB: u8 = 0x1F;

/// Quaternion data
pub(crate) const BNO055_QUA_DATA_W_LSB: u8 = 0x20;
pub(crate) const BNO055_QUA_DATA_W_MSB: u8 = 0x21;
pub(crate) const BNO055_QUA_DATA_X_LSB: u8 = 0x22;
pub(crate) const BNO055_QUA_DATA_X_MSB: u8 = 0x23;
pub(crate) const BNO055_QUA_DATA_Y_LSB: u8 = 0x24;
pub(crate) const BNO055_QUA_DATA_Y_MSB: u8 = 0x25;
pub(crate) const BNO055_QUA_DATA_Z_LSB: u8 = 0x26;
pub(crate) const BNO055_QUA_DATA_Z_MSB: u8 = 0x27;

/// Linear acceleration data
pub(crate) const BNO055_LIA_DATA_X_LSB: u8 = 0x28;
pub(crate) const BNO055_LIA_DATA_X_MSB: u8 = 0x29;
pub(crate) const BNO055_LIA_DATA_Y_LSB: u8 = 0x2A;
pub(crate) const BNO055_LIA_DATA_Y_MSB: u8 = 0x2B;
pub(crate) const BNO055_LIA_DATA_Z_LSB: u8 = 0x2C;
pub(crate) const BNO055_LIA_DATA_Z_MSB: u8 = 0x2D;

/// Gravity vector data
pub(crate) const BNO055_GRV_DATA_X_LSB: u8 = 0x2E;
pub(crate) const BNO055_GRV_DATA_X_MSB: u8 = 0x2F;
pub(crate) const BNO055_GRV_DATA_Y_LSB: u8 = 0x30;
pub(crate) const BNO055_GRV_DATA_Y_MSB: u8 = 0x31;
pub(crate) const BNO055_GRV_DATA_Z_LSB: u8 = 0x32;
pub(crate) const BNO055_GRV_DATA_Z_MSB: u8 = 0x33;

/// Temperature data
pub(crate) const BNO055_TEMP: u8 = 0x34;

/// Calibration Status
pub(crate) const BNO055_CALIB_STAT: u8 = 0x35;

pub(crate) const BNO055_ST_RESULT: u8 = 0x36;
pub(crate) const BNO055_INT_STA: u8 = 0x37;
pub(crate) const BNO055_SYS_CLK_STATUS: u8 = 0x38;
pub(crate) const BNO055_SYS_STATUS: u8 = 0x39;
pub(crate) const BNO055_SYS_ERR: u8 = 0x3A;
pub(crate) const BNO055_UNIT_SEL: u8 = 0x3B;
pub(crate) const BNO055_OPR_MODE: u8 = 0x3D;
pub(crate) const BNO055_PWR_MODE: u8 = 0x3E;

pub(crate) const BNO055_SYS_TRIGGER: u8 = 0x3F;
pub(crate) const BNO055_SYS_TRIGGER_RST_SYS_BIT: u8 = 0x20; // Reset command
pub(crate) const BNO055_SYS_TRIGGER_SELF_TEST_BIT: u8 = 0b000_0001; // Self-test command
pub(crate) const BNO055_TEMP_SOURCE: u8 = 0x40;
pub(crate) const BNO055_AXIS_MAP_CONFIG: u8 = 0x41;
pub(crate) const BNO055_AXIS_MAP_SIGN: u8 = 0x42;

/// Calibration data

pub(crate) const BNO055_ACC_OFFSET_X_LSB: u8 = 0x55;
pub(crate) const BNO055_ACC_OFFSET_X_MSB: u8 = 0x56;
pub(crate) const BNO055_ACC_OFFSET_Y_LSB: u8 = 0x57;
pub(crate) const BNO055_ACC_OFFSET_Y_MSB: u8 = 0x58;
pub(crate) const BNO055_ACC_OFFSET_Z_LSB: u8 = 0x59;
pub(crate) const BNO055_ACC_OFFSET_Z_MSB: u8 = 0x5A;

pub(crate) const BNO055_MAG_OFFSET_X_LSB: u8 = 0x5B;
pub(crate) const BNO055_MAG_OFFSET_X_MSB: u8 = 0x5C;
pub(crate) const BNO055_MAG_OFFSET_Y_LSB: u8 = 0x5D;
pub(crate) const BNO055_MAG_OFFSET_Y_MSB: u8 = 0x5E;
pub(crate) const BNO055_MAG_OFFSET_Z_LSB: u8 = 0x5F;
pub(crate) const BNO055_MAG_OFFSET_Z_MSB: u8 = 0x60;

pub(crate) const BNO055_GYR_OFFSET_X_LSB: u8 = 0x61;
pub(crate) const BNO055_GYR_OFFSET_X_MSB: u8 = 0x62;
pub(crate) const BNO055_GYR_OFFSET_Y_LSB: u8 = 0x63;
pub(crate) const BNO055_GYR_OFFSET_Y_MSB: u8 = 0x64;
pub(crate) const BNO055_GYR_OFFSET_Z_LSB: u8 = 0x65;
pub(crate) const BNO055_GYR_OFFSET_Z_MSB: u8 = 0x66;

pub(crate) const BNO055_ACC_RADIUS_LSB: u8 = 0x67;
pub(crate) const BNO055_ACC_RADIUS_MSB: u8 = 0x68;
pub(crate) const BNO055_MAG_RADIUS_LSB: u8 = 0x69;
pub(crate) const BNO055_MAG_RADIUS_MSB: u8 = 0x6A;
