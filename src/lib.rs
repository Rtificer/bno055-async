#![doc(html_root_url = "https://docs.rs/bno055-async/0.1.0")]
#![cfg_attr(not(feature = "std"), no_std)]

//! Bosch Sensortec BNO055 9-axis IMU sensor driver.
//! Datasheet: <`/https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf`>
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

#[cfg(not(feature = "defmt-03"))]
use bitflags::bitflags;
#[cfg(feature = "defmt-03")]
use defmt::bitflags;

use byteorder::{ByteOrder, LittleEndian};
pub use mint;
use num_enum::{IntoPrimitive, TryFromPrimitive};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

mod acc_config;
mod regs;
#[cfg(feature = "std")]
mod std;

pub use acc_config::{AccBandwidth, AccConfig, AccGRange, AccOperationMode};
pub use regs::BNO055_ID;

use zerocopy::{FromBytes, IntoBytes};
#[allow(clippy::wildcard_imports)]
use zerocopy_derive::*;

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    /// Contains the read ID
    InvalidChipId(u8),

    /// Invalid (not applicable) device mode.
    InvalidMode,

    /// Accelerometer configuration error
    AccConfig(acc_config::Error),

    // Invalid axis map value was read
    // Contains the read value
    InvalidAxisMap(u8),

    // Invalid status code value was read
    // Contains the read value
    InvalidSystemStatusCode(u8),

    // Invalid error code value was read
    // Contains the read value
    InvalidSystemErrorCode(u8),

    // Invalid power mode value was read
    // Contains the read value
    InvalidPowerMode(u8),

    // Invalid system mode value read
    InvalidSysMode(u8),
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055<I> {
    i2c: I,
    mode: BNO055OperationMode,
    use_default_addr: bool,
    page: BNO055RegisterPage,
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
            mode: BNO055OperationMode::ConfigMode,
            use_default_addr: true,
            page: BNO055RegisterPage::Page0,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I {
        self.i2c
    }

    #[must_use]
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
    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let id = self.id().await?;
        if id != regs::BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset(delay).await?;
        self.set_mode(BNO055OperationMode::ConfigMode, delay)
            .await?;
        self.set_power_mode(BNO055PowerMode::Normal).await?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, 0x00)
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    pub async fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            regs::BNO055_SYS_TRIGGER_RST_SYS_BIT,
        )
        .await
        .map_err(Error::I2c)?;

        // As per table 1.2
        delay.delay_ms(650).await;
        Ok(())
    }

    /// Sets the operating mode, see [`BNO055OperationMode`]
    /// See section 3.3.
    pub async fn set_mode(
        &mut self,
        mode: BNO055OperationMode,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<E>> {
        if self.mode != mode {
            self.set_page(BNO055RegisterPage::Page0).await?;

            self.mode = mode;

            self.write_u8(regs::BNO055_OPR_MODE, u8::from(mode))
                .await
                .map_err(Error::I2c)?;

            // Table 3-6 says 19ms to switch to CONFIG_MODE
            delay.delay_ms(19).await;
        }

        Ok(())
    }

    /// Sets the power mode, see [`BNO055PowerMode`]
    /// See section 3.2
    pub async fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        self.write_u8(regs::BNO055_PWR_MODE, u8::from(mode))
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns BNO055's power mode.
    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let mode = self
            .read_u8(regs::BNO055_PWR_MODE)
            .await
            .map_err(Error::I2c)?;

        BNO055PowerMode::try_from_primitive(mode).map_err(|err| Error::InvalidPowerMode(err.number))
    }

    /// Enables/Disables usage of external 32k crystal.
    pub async fn set_external_crystal(
        &mut self,
        ext: bool,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let prev = self.mode;
        self.set_mode(BNO055OperationMode::ConfigMode, delay)
            .await?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, if ext { 0x80 } else { 0x00 })
            .await
            .map_err(Error::I2c)?;

        self.set_mode(prev, delay).await?;

        Ok(())
    }

    /// Configures axis remap of the device.
    pub async fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let remap_value = (u8::from(remap.x) & 0b11)
            | ((u8::from(remap.y) & 0b11) << 2)
            | ((u8::from(remap.z) & 0b11) << 4);

        self.write_u8(regs::BNO055_AXIS_MAP_CONFIG, remap_value)
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns axis remap of the device.
    pub async fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_CONFIG)
            .await
            .map_err(Error::I2c)?;

        let remap = AxisRemap {
            x: BNO055AxisConfig::try_from_primitive(value & 0b11)
                .map_err(|err| Error::InvalidAxisMap(err.number))?,
            y: BNO055AxisConfig::try_from_primitive((value >> 2) & 0b11)
                .map_err(|err| Error::InvalidAxisMap(err.number))?,
            z: BNO055AxisConfig::try_from_primitive((value >> 4) & 0b11)
                .map_err(|err| Error::InvalidAxisMap(err.number))?,
        };

        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub async fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        self.write_u8(regs::BNO055_AXIS_MAP_SIGN, sign.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Return device's axes sign.
    pub async fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_SIGN)
            .await
            .map_err(Error::I2c)?;

        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope of
    /// the BNO055 device.
    pub async fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(regs::BNO055_ACC_ID, &mut buf)
            .await
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
    pub async fn get_system_status(
        &mut self,
        do_selftest: bool,
        delay: &mut impl DelayNs,
    ) -> Result<BNO055SystemStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::ConfigMode, delay)
                .await?;

            let result = async {
                let sys_trigger = self
                    .read_u8(regs::BNO055_SYS_TRIGGER)
                    .await
                    .map_err(Error::I2c)?;

                self.write_u8(regs::BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                    .await
                    .map_err(Error::I2c)?;

                // Wait for self-test result
                delay.delay_ms(400).await;

                let result = self
                    .read_u8(regs::BNO055_ST_RESULT)
                    .await
                    .map_err(Error::I2c)?;

                Ok(BNO055SelfTestStatus::from_bits_truncate(result))
            }
            .await;

            self.set_mode(prev, delay).await?; // Restore previous mode
            Some(result?)
        } else {
            None
        };

        let status = self
            .read_u8(regs::BNO055_SYS_STATUS)
            .await
            .map_err(Error::I2c)?;
        let error = self
            .read_u8(regs::BNO055_SYS_ERR)
            .await
            .map_err(Error::I2c)?;

        Ok(BNO055SystemStatus {
            status: BNO055SystemStatusCode::try_from_primitive(status)
                .map_err(|err| Error::InvalidSystemStatusCode(err.number))?,
            error: BNO055SystemErrorCode::try_from_primitive(error)
                .map_err(|err| Error::InvalidSystemErrorCode(err.number))?,
            selftest,
        })
    }

    /// Gets a quaternion (`mint::Quaternion<f32>`) reading from the BNO055.
    /// Available only in sensor fusion modes.
    pub async fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        // Device should be in fusion mode to be able to produce quaternions
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
                .await
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let x = f32::from(x) * scale;
            let y = f32::from(y) * scale;
            let z = f32::from(z) * scale;
            let w = f32::from(w) * scale;

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
    pub async fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        // Device should be in fusion mode to be able to produce Euler angles
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 6] = [0; 6];

            self.read_bytes(regs::BNO055_EUL_HEADING_LSB, &mut buf)
                .await
                .map_err(Error::I2c)?;

            let heading = f32::from(LittleEndian::read_i16(&buf[0..2]));
            let roll = f32::from(LittleEndian::read_i16(&buf[2..4]));
            let pitch = f32::from(LittleEndian::read_i16(&buf[4..6]));

            let scale = 1f32 / 16f32; // 1 degree = 16 LSB

            let rot = mint::EulerAngles::from([roll * scale, pitch * scale, heading * scale]);

            Ok(rot)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get calibration status
    #[must_use]
    pub async fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let status = self
            .read_u8(regs::BNO055_CALIB_STAT)
            .await
            .map_err(Error::I2c)?;

        let sys = (status >> 6) & 0b11;
        let gyr = (status >> 4) & 0b11;
        let acc = (status >> 2) & 0b11;
        let mag = status & 0b11;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    /// Checks whether device is fully calibrated or not.
    #[must_use]
    pub async fn is_fully_calibrated(&mut self) -> Result<bool, Error<E>> {
        let status = self.get_calibration_status().await?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    /// Reads current calibration profile of the device.
    pub async fn calibration_profile(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<BNO055Calibration, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::ConfigMode, delay)
            .await?;

        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];

        self.read_bytes(regs::BNO055_ACC_OFFSET_X_LSB, &mut buf[..])
            .await
            .map_err(Error::I2c)?;

        let res = BNO055Calibration::read_from_bytes(&buf).unwrap();

        self.set_mode(prev_mode, delay).await?;

        Ok(res)
    }

    /// Sets current calibration profile.
    pub async fn set_calibration_profile(
        &mut self,
        calib: &BNO055Calibration,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::ConfigMode, delay)
            .await?;

        // Combine register address and profile into single buffer
        let mut buf_with_reg = [0u8; 1 + BNO055_CALIB_SIZE];
        buf_with_reg[0] = regs::BNO055_ACC_OFFSET_X_LSB;
        buf_with_reg[1..].copy_from_slice(calib.as_bytes());

        self.i2c
            .write(self.i2c_addr(), &buf_with_reg[..])
            .await
            .map_err(Error::I2c)?;

        self.set_mode(prev_mode, delay).await?;

        Ok(())
    }

    /// Returns device's factory-programmed and constant chip ID.
    /// This ID is device model ID and not a BNO055's unique ID, which is stored in different register.
    #[must_use]
    pub async fn id(&mut self) -> Result<u8, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;
        self.read_u8(regs::BNO055_CHIP_ID).await.map_err(Error::I2c)
    }

    /// Updates struct internal mode value and reads mode from device
    pub async fn sync_mode(&mut self) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        let mode = self
            .read_u8(regs::BNO055_OPR_MODE)
            .await
            .map_err(Error::I2c)?;

        let mode = BNO055OperationMode::try_from_primitive(mode)
            .map_err(|err| Error::InvalidSysMode(err.number))?;

        self.mode = mode;

        Ok(())
    }

    /// Checks whether the device is in Sensor Fusion mode or not.
    pub fn is_in_fusion_mode(&self) -> Result<bool, Error<E>> {
        Ok(self.mode.is_fusion_enabled())
    }

    pub async fn get_acc_config(&mut self) -> Result<AccConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::Page1).await?;

        let bits = self
            .read_u8(regs::BNO055_ACC_CONFIG)
            .await
            .map_err(Error::I2c)?;

        let acc_config = AccConfig::try_from_bits(bits).map_err(Error::AccConfig)?;

        Ok(acc_config)
    }

    pub async fn set_acc_config(&mut self, acc_config: &AccConfig) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::Page1).await?;

        self.write_u8(regs::BNO055_ACC_CONFIG, acc_config.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets current register map page.
    async fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        if page != self.page {
            self.write_u8(regs::BNO055_PAGE_ID, u8::from(page))
                .await
                .map_err(Error::I2c)?;

            self.page = page;
        }
        Ok(())
    }

    /// Reads a vector of sensor data from the device.
    async fn read_vec_raw(&mut self, reg: u8) -> Result<mint::Vector3<i16>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(reg, &mut buf).await.map_err(Error::I2c)?;

        let x = LittleEndian::read_i16(&buf[0..2]);
        let y = LittleEndian::read_i16(&buf[2..4]);
        let z = LittleEndian::read_i16(&buf[4..6]);

        Ok(mint::Vector3::from([x, y, z]))
    }

    /// Applies the given scaling to the vector of sensor data from the device.
    fn scale_vec(raw: mint::Vector3<i16>, scaling: f32) -> mint::Vector3<f32> {
        mint::Vector3::from([
            f32::from(raw.x) * scaling,
            f32::from(raw.y) * scaling,
            f32::from(raw.z) * scaling,
        ])
    }

    /// Returns linear acceleration vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn linear_acceleration_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::Page0).await?;
            self.read_vec_raw(regs::BNO055_LIA_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns linear acceleration vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let linear_acceleration = self.linear_acceleration_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(linear_acceleration, scaling))
    }

    /// Returns gravity vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn gravity_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::Page0).await?;
            self.read_vec_raw(regs::BNO055_GRV_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns gravity vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let gravity = self.gravity_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(gravity, scaling))
    }

    /// Returns current accelerometer data in cm/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub async fn accel_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_accel_enabled() {
            self.set_page(BNO055RegisterPage::Page0).await?;
            self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current accelerometer data in m/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub async fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let a = self.accel_data_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(a, scaling))
    }

    /// Returns current gyroscope data in 1/16th deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub async fn gyro_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_gyro_enabled() {
            self.set_page(BNO055RegisterPage::Page0).await?;
            self.read_vec_raw(regs::BNO055_GYR_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current gyroscope data in deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub async fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let g = self.gyro_data_fixed().await?;
        let scaling = 1f32 / 16f32; // 1 deg/s = 16 lsb
        Ok(Self::scale_vec(g, scaling))
    }

    /// Returns current magnetometer data in 1/16th uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub async fn mag_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_mag_enabled() {
            self.set_page(BNO055RegisterPage::Page0).await?;
            self.read_vec_raw(regs::BNO055_MAG_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current magnetometer data in uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub async fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let m = self.mag_data_fixed().await?;
        let scaling = 1f32 / 16f32; // 1 uT = 16 lsb
        Ok(Self::scale_vec(m, scaling))
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    pub async fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::Page0).await?;

        // Read temperature signed byte
        let temp = self.read_u8(regs::BNO055_TEMP).await.map_err(Error::I2c)? as i8;
        Ok(temp)
    }

    #[inline(always)]
    fn i2c_addr(&self) -> u8 {
        if self.use_default_addr {
            regs::BNO055_DEFAULT_ADDR
        } else {
            regs::BNO055_ALTERNATE_ADDR
        }
    }

    pub fn mode(&self) -> BNO055OperationMode {
        self.mode
    }

    async fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte = [0u8; 1];
        self.i2c
            .write_read(self.i2c_addr(), &[reg], &mut byte)
            .await?;

        Ok(byte[0])
    }

    async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(), &[reg], buf).await
    }

    async fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(), &[reg, value]).await?;

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055AxisConfig {
    AxisAsX = 0b00,
    AxisAsY = 0b01,
    AxisAsZ = 0b10,
}

impl AxisRemap {
    #[must_use]
    pub fn x(&self) -> BNO055AxisConfig {
        self.x
    }

    #[must_use]
    pub fn y(&self) -> BNO055AxisConfig {
        self.y
    }

    #[must_use]
    pub fn z(&self) -> BNO055AxisConfig {
        self.z
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AxisRemap {
    x: BNO055AxisConfig,
    y: BNO055AxisConfig,
    z: BNO055AxisConfig,
}

impl AxisRemap {
    #[must_use]
    pub fn new(x: BNO055AxisConfig, y: BNO055AxisConfig, z: BNO055AxisConfig) -> Option<Self> {
        if x == y || y == z || z == x {
            None
        } else {
            Some(Self { x, y, z })
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055SystemStatusCode {
    SystemIdle = 0,
    SystemError = 1,
    InitPeripherals = 2,
    SystemInit = 3,
    Executing = 4,
    Running = 5,
    RunningWithoutFusion = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055SystemErrorCode {
    None = 0,
    PeripheralInit = 1,
    SystemInit = 2,
    SelfTest = 3,
    RegisterMapValue = 4,
    RegisterMapAddress = 5,
    RegisterMapWrite = 6,
    LowPowerModeNotAvail = 7,
    AccelPowerModeNotAvail = 8,
    FusionAlgoConfig = 9,
    SensorConfig = 10,
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
    pub status: BNO055SystemStatusCode,
    pub selftest: Option<BNO055SelfTestStatus>,
    pub error: BNO055SystemErrorCode,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoBytes, FromBytes, KnownLayout, Immutable)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055RegisterPage {
    Page0 = 0,
    Page1 = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055PowerMode {
    Normal = 0,
    LowPower = 1,
    Suspend = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum BNO055OperationMode {
    ConfigMode = 0b0000,
    AccOnly = 0b0001,
    MagOnly = 0b0010,
    GyroOnly = 0b0011,
    AccMag = 0b0100,
    AccGyro = 0b0101,
    MagGyro = 0b0110,
    AMG = 0b0111,
    IMU = 0b1000,
    Compass = 0b1001,
    M4G = 0b1010,
    NdofFmcOff = 0b1011,
    NDOF = 0b1100,
}

impl BNO055OperationMode {
    fn is_fusion_enabled(self) -> bool {
        matches!(
            self,
            Self::IMU | Self::Compass | Self::M4G | Self::NdofFmcOff | Self::NDOF,
        )
    }

    fn is_accel_enabled(self) -> bool {
        matches!(
            self,
            Self::AccOnly
                | Self::AccMag
                | Self::AccGyro
                | Self::AMG
                | Self::IMU
                | Self::Compass
                | Self::M4G
                | Self::NdofFmcOff
                | Self::NDOF,
        )
    }

    fn is_gyro_enabled(self) -> bool {
        matches!(
            self,
            Self::GyroOnly
                | Self::AccGyro
                | Self::MagGyro
                | Self::AMG
                | Self::IMU
                | Self::NdofFmcOff
                | Self::NDOF,
        )
    }

    fn is_mag_enabled(self) -> bool {
        matches!(
            self,
            Self::MagOnly
                | Self::AccMag
                | Self::MagGyro
                | Self::AMG
                | Self::Compass
                | Self::M4G
                | Self::NdofFmcOff
                | Self::NDOF,
        )
    }
}
