use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

#[allow(clippy::unusual_byte_groupings)]
const ACC_G_RANGE_MASK: u8 = 0b000_000_11;
#[allow(clippy::unusual_byte_groupings)]
const ACC_BANDWIDTH_MASK: u8 = 0b000_111_00;
#[allow(clippy::unusual_byte_groupings)]
const ACC_OPERATION_MODE_MASK: u8 = 0b111_000_00;

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)]
pub enum Error {
    BadAccGRange,
    BadAccBandwidth,
    BadAccOperationMode,
}

#[derive(Debug, Clone, Copy, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccGRange {
    G2 = 0b000_000_00,
    G4 = 0b000_000_01,
    G8 = 0b000_000_10,
    G16 = 0b000_000_11,
}

#[derive(Debug, Clone, Copy, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccBandwidth {
    Hz7_81 = 0b000_000_00,
    Hz15_63 = 0b000_001_00,
    Hz31_25 = 0b000_010_00,
    Hz62_5 = 0b000_011_00,
    Hz125 = 0b000_100_00,
    Hz250 = 0b000_101_00,
    Hz500 = 0b000_110_00,
    Hz1000 = 0b000_111_00,
}

#[derive(Debug, Clone, Copy, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccOperationMode {
    Normal = 0b000_000_00,
    Suspend = 0b001_000_00,
    LowPower1 = 0b010_000_00,
    Standby = 0b011_000_00,
    LowPower2 = 0b100_000_00,
    DeepSuspend = 0b101_000_00,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AccConfig {
    g_range: AccGRange,
    bandwidth: AccBandwidth,
    operation_mode: AccOperationMode,
}

impl AccConfig {
    pub fn try_from_bits(bits: u8) -> Result<Self, Error> {
        let g_range = AccGRange::from_u8(bits & ACC_G_RANGE_MASK).ok_or(Error::BadAccGRange)?;
        let bandwidth =
            AccBandwidth::from_u8(bits & ACC_BANDWIDTH_MASK).ok_or(Error::BadAccBandwidth)?;
        let operation_mode = AccOperationMode::from_u8(bits & ACC_OPERATION_MODE_MASK)
            .ok_or(Error::BadAccOperationMode)?;

        Ok(Self {
            g_range,
            bandwidth,
            operation_mode,
        })
    }

    pub fn bits(&self) -> u8 {
        self.operation_mode as u8 | self.bandwidth as u8 | self.g_range as u8
    }

    pub fn g_range(&self) -> AccGRange {
        self.g_range
    }

    pub fn bandwidth(&self) -> AccBandwidth {
        self.bandwidth
    }

    pub fn operation_mode(&self) -> AccOperationMode {
        self.operation_mode
    }

    pub fn set_g_range(&mut self, g_range: AccGRange) {
        self.g_range = g_range;
    }

    pub fn set_bandwidth(&mut self, bandwidth: AccBandwidth) {
        self.bandwidth = bandwidth;
    }

    pub fn set_operation_mode(&mut self, operation_mode: AccOperationMode) {
        self.operation_mode = operation_mode;
    }
}
