use rppal::i2c::I2c;
use std::time::{Duration, Instant};
use std::f32::consts::PI;
use std::ops::{Add, Div};

const MPU6050_ADDR: u16 = 0x68; // I2C address of the MPU6050
const ACCEL_CAL_TIME: u16 = 3500; // Default accelerometer calibration time in milliseconds

fn main() {
  let mut i2c = init_mpu6050(); // Set up I2C device (the GY-521 accelerometer/gyro)

  let (accel_offsets, gyro_offsets) = calibrate_mpu6050(&i2c); // Calibrate the accelerometer

  loop {
    let accel_point = get_acceleration(&mut i2c);
    let zeroed_accel_point @ DataPoint {x: a_x, y: a_y, z: a_z} = accel_point - accel_offsets;

    let gyro_point = get_gyroscope(&mut i2c);
    let zeroed_gyro_point @ DataPoint {x: g_x, y: g_y, z: g_z} = gyro_point - gyro_offsets;

    print!("Accel: X={},\tY={},\tZ={}\t", a_x, a_y, a_z);
    println!("Gyro: X={},\tY={},\tZ={}", g_x, g_y, g_z);
  }
}


fn init_mpu6050() -> I2c {
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
fn calibrate_mpu6050(i2c: &mut I2c, max_calibration_time: Option<Duration>, 
                     diff_between_iters: Option<f32>, consistent_iters: Option<u8>)
                     -> (DataPoint, DataPoint) {
  // set default values if none given
  let max_calibration_time = max_calibration_time.unwrap_or(Duration::from_millis(ACCEL_CAL_TIME));
  let diff_between_iters = diff_between_iters.unwrap_or(2.0);
  let consistent_iters = consistent_iters.unwrap_or(5);

  // vector of data points to average
  let mut accel_vec: Vec<DataPoint> = Vec::new();
  let mut gyro_vec:  Vec<DataPoint> = Vec::new();

  // previous average values (to see if current iteration is consistent with previous)
  let mut prev_avg_accel = DataPoint::default();
  let mut prev_avg_gyro  = DataPoint::default();

  // whether or not the average has been found during loop (to not keep evaluating)
  let mut avg_accel_found = false;
  let mut avg_gyro_found  = false;

  // number of iterations where the average has been consistent
  let mut consistent_accel_iters = 0;
  let mut consistent_gyro_iters  = 0;

  // final values to be used for calibration
  let mut avg_accel_offset = get_acceleration(&mut i2c);
  let mut avg_gyro_offset  = get_gyroscope(&mut i2c);

  let start = Instant::now();

  loop {
    // calculate acceleration (if needed)
    if !avg_accel_found {
      let accel_point = get_acceleration(&mut i2c);
      accel_vec.push(accel_point);
      let avg = get_average(&accel_vec);
      if (prev_avg_accel.x - avg.x).abs() < diff_between_iters &&
         (prev_avg_accel.y - avg.y).abs() < diff_between_iters &&
         (prev_avg_accel.z - avg.z).abs() < diff_between_iters {
        // average didn't change much between iterations
        consistent_accel_iters += 1;
      } else {
        consistent_accel_iters = 0; // reset counter
      }
      prev_avg_accel = accel_point;
      if consistent_accel_iters >= consistent_iters {
        avg_accel_found = true; // don't keep evaluating acceleration
        avg_accel_offset = avg; // set final calibration values
      }
    }
    
    // calculate gyroscope (if needed)
    if !avg_gyro_found {
      let gyro_point = get_gyroscope(&mut i2c);
      gyro_vec.push(gyro_point);
      let avg = get_average(&gyro_vec);
      if (prev_avg_gyro.x - avg.x).abs() < diff_between_iters &&
         (prev_avg_gyro.y - avg.y).abs() < diff_between_iters &&
         (prev_avg_gyro.z - avg.z).abs() < diff_between_iters {
        // average didn't change much between iterations
        consistent_gyro_iters += 1;
      } else {
        consistent_gyro_iters = 0; // reset counter
      }
      prev_avg_gyro = gyro_point;
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

fn get_average(points: &[DataPoint]) -> DataPoint {
  let len = points.len() as i32;

  let (sum_x, sum_y, sum_z) = points.iter().fold((0, 0, 0), |acc, p| {
      (acc.0 + p.x as i32, acc.1 + p.y as i32, acc.2 + p.z as i32)
  });

  DataPoint {
      x: (sum_x / len) as i16,
      y: (sum_y / len) as i16,
      z: (sum_z / len) as i16,
  }
}


fn get_acceleration(i2c: &mut I2c) -> DataPoint {
  let mut accel_data = [0; 6];
  let _ = i2c.write_read(&[0x3B], &mut accel_data);

  let accel_x = i16::from_be_bytes([accel_data[0], accel_data[1]]);
  let accel_y = i16::from_be_bytes([accel_data[2], accel_data[3]]);
  let accel_z = i16::from_be_bytes([accel_data[4], accel_data[5]]);
  DataPoint {x: accel_x, y: accel_y, z: accel_z}
}

fn get_gyroscope(i2c: &mut I2c) -> DataPoint {
  let mut gyro_data = [0; 6];
  let _ = i2c.write_read(&[0x43], &mut gyro_data);

  let gyro_x = i16::from_be_bytes([gyro_data[0], gyro_data[1]]);
  let gyro_y = i16::from_be_bytes([gyro_data[2], gyro_data[3]]);
  let gyro_z = i16::from_be_bytes([gyro_data[4], gyro_data[5]]);
  DataPoint {x: gyro_x, y: gyro_y, z: gyro_z}
}


#[derive(Clone, Copy, Default, Debug)]
struct DataPoint {
  x: i16,
  y: i16,
  z: i16,
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


fn degrees_to_radians(degrees: f32) -> f32 {
    degrees * PI / 180.0
}

/// function to compute new position of a value in one direction
fn compute_position(x0: f32, v0: f32, a: f32, t: f32) -> f32 {
    // x = x0 + v0*t + 1/2*a*t^2
    let x = x0 + v0*t + 0.5*a*t.powi(2);
    x
}

/// function to compute new velocity of a value in one direction
fn compute_velocity(v0: f32, a: f32, t: f32) -> f32 {
    // v = v0 + a*t
    let v = v0 + a*t;
    v
}



#[cfg(test)]
mod tests {
  use super::*;
  use std::f32::consts::PI;

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
  fn test_degrees_to_radians() {
    let deg = 180.0;
    let rad = degrees_to_radians(deg);
    assert_eq!(rad, PI);
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