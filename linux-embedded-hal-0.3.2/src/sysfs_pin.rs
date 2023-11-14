//! Linux Sysfs pin type

use std::path::Path;

/// Newtype around [`sysfs_gpio::Pin`] that implements the `embedded-hal` traits
///
/// [`sysfs_gpio::Pin`]: https://docs.rs/sysfs_gpio/0.6.0/sysfs_gpio/struct.Pin.html
pub struct SysfsPin(pub sysfs_gpio::Pin);

impl SysfsPin {
    /// See [`sysfs_gpio::Pin::new`][0] for details.
    ///
    /// [0]: https://docs.rs/sysfs_gpio/0.6.0/sysfs_gpio/struct.Pin.html#method.new
    pub fn new(pin_num: u64) -> Self {
        SysfsPin(sysfs_gpio::Pin::new(pin_num))
    }

    /// See [`sysfs_gpio::Pin::from_path`][0] for details.
    ///
    /// [0]: https://docs.rs/sysfs_gpio/0.6.0/sysfs_gpio/struct.Pin.html#method.from_path
    pub fn from_path<P>(path: P) -> sysfs_gpio::Result<Self>
    where
        P: AsRef<Path>,
    {
        sysfs_gpio::Pin::from_path(path).map(SysfsPin)
    }
}

impl hal::digital::v2::OutputPin for SysfsPin {
    type Error = sysfs_gpio::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        if self.0.get_active_low()? {
            self.0.set_value(1)
        } else {
            self.0.set_value(0)
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        if self.0.get_active_low()? {
            self.0.set_value(0)
        } else {
            self.0.set_value(1)
        }
    }
}

impl hal::digital::v2::InputPin for SysfsPin {
    type Error = sysfs_gpio::Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        if !self.0.get_active_low()? {
            self.0.get_value().map(|val| val != 0)
        } else {
            self.0.get_value().map(|val| val == 0)
        }
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.is_high().map(|val| !val)
    }
}

impl core::ops::Deref for SysfsPin {
    type Target = sysfs_gpio::Pin;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl core::ops::DerefMut for SysfsPin {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
