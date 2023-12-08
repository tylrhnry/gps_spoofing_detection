use rppal::i2c::I2c;
use std::cell::RefMut;
use std::fmt::Display;
use std::ops::{Add, Sub, Div};
use std::time::{Duration, Instant};

const MPU6050_ADDR: u16 = 0x68; // I2C address of the MPU6050
const CALIB_TIME: u64 = 10000; // Default mpu6050 calibration time in milliseconds
const CALIB_DIFF: i16 = 2; // Default difference between iterations to be considered consistent
const CALIB_CONSISTENT: u8 = 5; // Default number of iterations that must be consistent to be considered the average
const GRAVITY_ACCEL: f32 = 9.80665; // Gravity acceleration in m/s^2
const ACCEL_SENSITIVITY: f32 = 16384.0; // Accelerometer sensitivity in LSB/g (Currently set to +-2g)



/// DataPoint struct to hold x, y, and z values for acceleration and gyroscope
#[derive(Clone, Copy, Default, Debug)]
pub struct DataPoint {
  x: i16,
  y: i16,
  z: i16,
}

#[derive(Clone, Copy, Debug)]
pub enum AccelPoint {
  Accel(DataPoint),
}

#[derive(Clone, Copy, Debug)]
pub enum GyroPoint {
  Gyro(DataPoint),
}

#[derive(Clone, Copy, Debug)]
pub struct RawPoint {
  x: f32,
  y: f32,
  z: f32,
}



/// Initializes the mpu6050 and returns an I2c device
pub fn init_mpu6050() -> I2c {
  let mut i2c = I2c::new().unwrap();
  i2c.set_slave_address(MPU6050_ADDR).unwrap();

  let _ = i2c.write_read(&[0x6B, 0x08], &mut [0; 1]); // enable the mpu6050

  let _ = i2c.write_read(&[0x1A, 0x05], &mut [0; 1]); // enable gyro low pass filter
  let _ = i2c.write_read(&[0x1B, 0x18], &mut [0; 1]); // set gyro range to +-1000 deg/s

  i2c
}


/// Iterates over mpu6050 data points to find average to be used to zero the
/// output.
/// 
/// Returns a tuple containing (acceleration offset, gyroscope offset) where
/// each is a Datapoint struct containing the x, y, and z offsets.
/// 
/// 'max_cal_time' is number of seconds to calibrate for
/// 'diff_between_iters' is the maximum difference between iterations to be considered consistent
/// 'consistent_iters' is the number of iterations that must be consistent to be considered the average
pub fn calibrate_mpu6050(i2c: RefMut<I2c>, max_calibration_time: Option<Duration>, 
                     diff_between_iters: Option<i16>, consistent_iters: Option<u8>)
                     -> (AccelPoint, GyroPoint) {
  // set default values if none given
  let max_calibration_time = max_calibration_time.unwrap_or(Duration::from_millis(CALIB_TIME));
  let diff_between_iters = diff_between_iters.unwrap_or(CALIB_DIFF);
  let consistent_iters = consistent_iters.unwrap_or(CALIB_CONSISTENT);

  // vector of data points to average
  let mut accel_vec: Vec<AccelPoint> = Vec::new();
  let mut gyro_vec:  Vec<GyroPoint> = Vec::new();

  // previous average values (to see if current iteration is consistent with previous)
  let mut prev_avg_accel = AccelPoint::default();
  let mut prev_avg_gyro  = GyroPoint::default();

  // whether or not the average has been found during loop (to not keep evaluating)
  let mut avg_accel_found = false;
  let mut avg_gyro_found  = false;

  // number of iterations where the average has been consistent
  let mut consistent_accel_iters = 0;
  let mut consistent_gyro_iters  = 0;

  // final values to be used for calibration
  let mut avg_accel_offset = get_acceleration(&i2c);
  let mut avg_gyro_offset  = get_gyroscope(&i2c);

  let start = Instant::now();

  loop {
    // calculate acceleration (if needed)
    if !avg_accel_found {
      let accel_point = get_acceleration(&i2c);
      accel_vec.push(accel_point);
      let avg = get_average(&accel_vec);

      if (prev_avg_accel.x() - avg.x()).abs() < diff_between_iters &&
          (prev_avg_accel.y() - avg.y()).abs() < diff_between_iters &&
          (prev_avg_accel.z() - avg.z()).abs() < diff_between_iters {
        // average didn't change much between iterations
        consistent_accel_iters += 1;
      } else {
        consistent_accel_iters = 0; // reset counter
      }
      prev_avg_accel = avg;
      if consistent_accel_iters >= consistent_iters {
        avg_accel_found = true; // don't keep evaluating acceleration
        // subtract 1g from z axis
        let gravity_avg = DataPoint::new(avg.x(), avg.y(), avg.z() - (ACCEL_SENSITIVITY as i16));
        avg_accel_offset = AccelPoint::Accel(gravity_avg); // set final calibration values
      }
    }
    
    // calculate gyroscope (if needed)
    if !avg_gyro_found {
      let gyro_point = get_gyroscope(&i2c);
      gyro_vec.push(gyro_point);
      let avg = get_average(&gyro_vec);

      if (prev_avg_gyro.x() - avg.x()).abs() < diff_between_iters &&
        (prev_avg_gyro.y() - avg.y()).abs() < diff_between_iters &&
        (prev_avg_gyro.z() - avg.z()).abs() < diff_between_iters {
        // average didn't change much between iterations
        consistent_gyro_iters += 1;
      } else {
        consistent_gyro_iters = 0; // reset counter
      }
      prev_avg_gyro = avg;
      if consistent_gyro_iters >= consistent_iters {
        avg_gyro_found = true; // don't keep evaluating acceleration
        avg_gyro_offset = avg; // set final calibration values
      }
    }

    // check if both averages have been found
    if avg_accel_found && avg_gyro_found {
      break;
    }
    // check if max time has been exceeded
    if start.elapsed() >= max_calibration_time {
      println!("Calibration time exceeded");
      break;
    };
  }

  (avg_accel_offset, avg_gyro_offset)
}

// this currently returns datapoint regardless of T being AccelPoint or GyroPoint
/// Returns the mean average of a vector of DataPoints
fn get_average<T>(points: &Vec<T>) -> T 
where
  T: DataPointType + Default
{

  if points.is_empty() {
      return T::default();
  }

  let len = points.len() as i32;

  let (sum_x, sum_y, sum_z) = points.iter().fold((0, 0, 0), |acc, p| {
      (acc.0 + p.x() as i32, acc.1 + p.y() as i32, acc.2 + p.z() as i32)
  });

  T::new (
      (sum_x / len) as i16,
      (sum_y / len) as i16,
      (sum_z / len) as i16,
  )
}


/// Reads mpu6050 and returns the acceleration data as a DataPoint struct
fn get_acceleration(i2c: &RefMut<I2c>) -> AccelPoint {
  let mut accel_data = [0; 6];
  let _ = i2c.write_read(&[0x3B], &mut accel_data);

  let accel_x = i16::from_be_bytes([accel_data[0], accel_data[1]]);
  let accel_y = i16::from_be_bytes([accel_data[2], accel_data[3]]);
  let accel_z = i16::from_be_bytes([accel_data[4], accel_data[5]]);
  AccelPoint::Accel(DataPoint {x: accel_x, y: accel_y, z: accel_z})
}

/// Reads mpu6050 and returns the gyroscope data as a DataPoint struct
fn get_gyroscope(i2c: &RefMut<I2c>) -> GyroPoint {
  let mut gyro_data = [0; 6];
  let _ = i2c.write_read(&[0x43], &mut gyro_data);

  let gyro_x = i16::from_be_bytes([gyro_data[0], gyro_data[1]]);
  let gyro_y = i16::from_be_bytes([gyro_data[2], gyro_data[3]]);
  let gyro_z = i16::from_be_bytes([gyro_data[4], gyro_data[5]]);
  GyroPoint::Gyro(DataPoint {x: gyro_x, y: gyro_y, z: gyro_z})
}


/// Converts raw acceleration data to m/s^2
fn convert_raw_point(raw_point: AccelPoint) -> RawPoint {
  match raw_point {
    AccelPoint::Accel(accel_data) => {
      let x = accel_data.x as f32 / ACCEL_SENSITIVITY * GRAVITY_ACCEL;
      let y = accel_data.y as f32 / ACCEL_SENSITIVITY * GRAVITY_ACCEL;
      let z = accel_data.z as f32 / ACCEL_SENSITIVITY * GRAVITY_ACCEL;
      RawPoint {x, y, z}
    }
  }
}

/// Gets an acceleration point, applies the calibration offsets, and converts to m/s^2
pub fn get_converted_acceleration(i2c: &RefMut<I2c>, accel_offsets: &AccelPoint) -> RawPoint {
  let accel_point = get_acceleration(&i2c);
  let offset_acceleration = accel_point - *accel_offsets;
  convert_raw_point(offset_acceleration)
}

/// Gets a gyroscope point and applies the calibration offsets
pub fn get_offset_gyroscope(i2c: &RefMut<I2c>, gyro_offsets: &GyroPoint) -> GyroPoint {
  let gyro_point = get_gyroscope(&i2c);
  gyro_point - *gyro_offsets
}




trait DataPointType {
  fn new(x: i16, y: i16, z: i16) -> Self;
  fn x(&self) -> i16;
  fn y(&self) -> i16;
  fn z(&self) -> i16;
}


/// DataPoint Implementations

impl DataPointType for DataPoint {
  fn new(x: i16, y: i16, z: i16) -> Self {
    DataPoint {x, y, z}
  }

  fn x(&self) -> i16 {
    self.x
  }

  fn y(&self) -> i16 {
    self.y
  }

  fn z(&self) -> i16 {
    self.z
  }
}

impl Add for DataPoint {
  type Output = Self;

  fn add(self, other: Self) -> Self {
    DataPoint {
      x: self.x + other.x,
      y: self.y + other.y,
      z: self.z + other.z,
    }
  }
}

impl Sub for DataPoint {
  type Output = Self;

  fn sub(self, other: Self) -> Self {
    DataPoint {
      x: self.x - other.x,
      y: self.y - other.y,
      z: self.z - other.z,
    }
  }
}

impl Div<i16> for DataPoint {
  type Output = Self;

  fn div(self, divisor: i16) -> Self {
    DataPoint {
      x: self.x / divisor,
      y: self.y / divisor,
      z: self.z / divisor,
    }
  }
}


/// Acceleration Implementations

impl DataPointType for AccelPoint {
  fn new(x: i16, y: i16, z: i16) -> Self {
    AccelPoint::Accel(DataPoint::new(x, y, z))
  }

  fn x(&self) -> i16 {
    match self {
      AccelPoint::Accel(accel_data) => accel_data.x,
    }
  }

  fn y(&self) -> i16 {
    match self {
      AccelPoint::Accel(accel_data) => accel_data.y,
    }
  }

  fn z(&self) -> i16 {
    match self {
      AccelPoint::Accel(accel_data) => accel_data.z,
    }
  }
}

impl Default for AccelPoint {
  fn default() -> Self {
    AccelPoint::Accel(DataPoint::default())
  }
}

impl Display for AccelPoint {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      AccelPoint::Accel(accel_data) => write!(f, "Accel: (x: {}, y: {}, z: {})", accel_data.x, accel_data.y, accel_data.z),
    }
  }
}

impl Add for AccelPoint {
  type Output = Self;

  fn add(self, other: Self) -> Self {
    match (self, other) {
      (AccelPoint::Accel(a1), AccelPoint::Accel(a2)) => AccelPoint::Accel(a1 + a2),
    }
  }
}

impl Sub for AccelPoint {
  type Output = Self;

  fn sub(self, other: Self) -> Self {
    match (self, other) {
      (AccelPoint::Accel(a1), AccelPoint::Accel(a2)) => AccelPoint::Accel(a1 - a2),
    }
  }
}

impl Div<i16> for AccelPoint {
  type Output = Self;

  fn div(self, divisor: i16) -> Self {
    match self {
      AccelPoint::Accel(a) => AccelPoint::Accel(a / divisor),
    }
  }
}


/// Gyroscope Implementations

impl DataPointType for GyroPoint {
  fn new(x: i16, y: i16, z: i16) -> Self {
    GyroPoint::Gyro(DataPoint::new(x, y, z))
  }

  fn x(&self) -> i16 {
    match self {
      GyroPoint::Gyro(gyro_data) => gyro_data.x,
    }
  }

  fn y(&self) -> i16 {
    match self {
      GyroPoint::Gyro(gyro_data) => gyro_data.y,
    }
  }

  fn z(&self) -> i16 {
    match self {
      GyroPoint::Gyro(gyro_data) => gyro_data.z,
    }
  }
}

impl Default for GyroPoint {
  fn default() -> Self {
    GyroPoint::Gyro(DataPoint::default())
  }
}

impl Display for GyroPoint {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      GyroPoint::Gyro(gyro_data) => write!(f, "Gyro:  (x: {}, y: {}, z: {})", gyro_data.x, gyro_data.y, gyro_data.z),
    }
  }
}

impl Add for GyroPoint {
  type Output = Self;

  fn add(self, other: Self) -> Self {
    match (self, other) {
      (GyroPoint::Gyro(g1), GyroPoint::Gyro(g2)) => GyroPoint::Gyro(g1 + g2),
    }
  }
}

impl Sub for GyroPoint {
  type Output = Self;

  fn sub(self, other: Self) -> Self {
    match (self, other) {
      (GyroPoint::Gyro(g1), GyroPoint::Gyro(g2)) => GyroPoint::Gyro(g1 - g2),
    }
  }
}

impl Div<i16> for GyroPoint {
  type Output = Self;

  fn div(self, divisor: i16) -> Self {
    match self {
      GyroPoint::Gyro(g) => GyroPoint::Gyro(g / divisor),
    }
  }
}


/// Raw Acceleration Implementations
impl Display for RawPoint {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "Accel: (x: {:.3}, y: {:.3}, z: {:.3})", self.x, self.y, self.z)
  }
}

impl RawPoint {
  pub fn new(x: f32, y: f32, z: f32) -> Self {
    RawPoint {x, y, z}
  }

  pub fn x(&self) -> f32 {
    self.x
  }

  pub fn y(&self) -> f32 {
    self.y
  }

  pub fn z(&self) -> f32 {
    self.z
  }
}




#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_get_average() {
    let points = vec![
      DataPoint {x: 1, y: 2, z: 3},
      DataPoint {x: 2, y: 3, z: 4},
      DataPoint {x: 3, y: 4, z: 5},
      DataPoint {x: 4, y: 5, z: 6},
      DataPoint {x: 5, y: 6, z: 7},
    ];
    let avg = get_average(&points);
    assert_eq!(avg.x, 3);
    assert_eq!(avg.y, 4);
    assert_eq!(avg.z, 5);
  }

  #[test]
  fn test_add_datapoint() {
    let p1 = DataPoint {x: 1, y: 2, z: 3};
    let p2 = DataPoint {x: 2, y: 3, z: 4};
    let p3 = p1 + p2;
    assert_eq!(p3.x, 3);
    assert_eq!(p3.y, 5);
    assert_eq!(p3.z, 7);
  }

  #[test]
  fn test_sub_datapoint() {
    let p1 = DataPoint {x: 1, y: 2, z: 3};
    let p2 = DataPoint {x: 2, y: 3, z: 4};
    let p3 = p2 - p1;
    assert_eq!(p3.x, 1);
    assert_eq!(p3.y, 1);
    assert_eq!(p3.z, 1);
  }

  #[test]
  fn test_div_datapoint() {
    let p1 = DataPoint {x: 12, y: 24, z: 36};
    let p2 = p1 / 2;
    assert_eq!(p2.x, 6);
    assert_eq!(p2.y, 12);
    assert_eq!(p2.z, 18);
  }

  #[test]
  fn test_compute_position() {
    let x0 = 0.0;
    let v0 = 0.0;
    let a = 0.0;
    let t = 0.0;
    let x = compute_position(x0, v0, a, t);
    assert_eq!(x, 0.0);

    let x0 = 0.0;
    let v0 = 0.0;
    let a = 5.0;
    let t = 2.0;
    let x = compute_position(x0, v0, a, t);
    assert_eq!(x, 10.0);

    let x0 = 10.0;
    let v0 = 5.0;
    let a = 5.0;
    let t = 2.0;
    let x = compute_position(x0, v0, a, t);
    assert_eq!(x, 30.0);
  }

  #[test]
  fn test_compute_velocity() {
    let v0 = 0.0;
    let a = 0.0;
    let t = 0.0;
    let v = compute_velocity(v0, a, t);
    assert_eq!(v, 0.0);

    let v0 = 0.0;
    let a = 5.0;
    let t = 2.0;
    let v = compute_velocity(v0, a, t);
    assert_eq!(v, 10.0);

    let v0 = 5.0;
    let a = 5.0;
    let t = 2.0;
    let v = compute_velocity(v0, a, t);
    assert_eq!(v, 15.0);
  }

}