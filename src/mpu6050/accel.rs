use rppal::i2c::I2c;
use std::time::{Duration, Instant};
use std::ops;

const MPU6050_ADDR: u16 = 0x68;

fn main() {
  // Set up I2C device (the GY-521 accelerometer/gyro)
  let mut i2c = I2c::new().unwrap();
  i2c.set_slave_address(MPU6050_ADDR).unwrap();

  let _ = i2c.write_read(&[0x6B, 0x08], &mut [0; 1]);

  // Read accelerometer values
  let start = Instant::now();
  let mut iters = 0;

  let accel_vec: Vec<AccelPoint> = Vec::new();
  let gyro_vec:  Vec<GyroPoint>  = Vec::new();

  loop {
    
    if start.elapsed() >= Duration::from_secs(1) {
      println!("{iters}");
      break;
    };

    let mut accel_data = [0; 6];
    let mut gyro_data  = [0; 6];
    let _ = i2c.write_read(&[0x3B], &mut accel_data);
    let _ = i2c.write_read(&[0x43], &mut gyro_data);
 

    let accel_x = i16::from_be_bytes([accel_data[0], accel_data[1]]);
    let accel_y = i16::from_be_bytes([accel_data[2], accel_data[3]]);
    let accel_z = i16::from_be_bytes([accel_data[4], accel_data[5]]);
    let accel_point = AccelPoint {x: accel_x, y: accel_y, z: accel_z};
    accel_vec.push(accel_point);

    let gyro_x = i16::from_be_bytes([gyro_data[0], gyro_data[1]]);
    let gyro_y = i16::from_be_bytes([gyro_data[2], gyro_data[3]]);
    let gyro_z = i16::from_be_bytes([gyro_data[4], gyro_data[5]]);
    let gyro_point = GyroPoint {x: gyro_x, y: gyro_y, z: gyro_z};
    gyro_vec.push(gyro_point);


    println!("Accelerometer: X={},\tY={},\tZ={}", accel_x, accel_y, accel_z);
    println!("Gyroscope:     X={},\tY={},\tZ={}", gyro_x, gyro_y, gyro_z);
    iters += 1;
  }

  println!("Average accel: {}", get_average(&accel_vec));
  println!("Average gyro:  {}", get_average(&gyro_vec));
}

struct AccelPoint {
  x: f32,
  y: f32,
  z: f32,
}

struct GyroPoint {
  x: f32,
  y: f32,
  z: f32,
}

fn get_average<T: MotionPoint>(points: &[T]) -> T {
  let len = points.len() as f32;
  let mut sum = points.iter().fold(T::default(), |acc, p| acc + p.clone());
  sum / len
}

trait MotionPoint: Clone + Default {
  fn default() -> Self;
  fn ops::add(&self, other: &Self) -> Self;
  fn ops::div(&self, divisor: f32) -> Self;
}


impl MotionPoint for AccelPoint {
  fn default() -> Self {
    AccelPoint {
      x: 0.0,
      y: 0.0,
      z: 0.0,
    }
  }

  fn ops::add(&self, other: &Self) -> Self {
    AccelPoint {
      x: self.x + other.x,
      y: self.y + other.y,
      z: self.z + other.z,
    }
  }

  fn ops::div(&self, divisor: f32) -> Self {
    AccelPoint {
      x: self.x / divisor,
      y: self.y / divisor,
      z: self.z / divisor,
    }
  }
}


impl MotionPoint for GyroPoint {
  fn default() => Self {
    GyroPoint {
      x: 0.0,
      y: 0.0,
      z: 0.0,
    }
  }

  fn ops::add(&self, other: &Self) -> Self {
    GyroPoint {
      x: self.x + other.x,
      y: self.y + other.y,
      z: self.z + other.z,
    }
  }

  fn ops::div(&self, divisor: f32) -> Self {
    GyroPoint {
      x: self.x / divisor,
      y: self.y / divisor,
      z: self.z / divisor,
    }
  }
}


impl pos_data {
    fn new() -> pos_data { // modify this to take in the data from the GPS for starting pos.
        pos_data {
            pos_x: 0.0,
            pos_y: 0.0,
            pos_z: 0.0,
            vel_x: 0.0,
            vel_y: 0.0,
            vel_z: 0.0,
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
            time: 0.0,
        }
    }
}


/// calibrates the accelerometer at the start of the program
/// by taking the average of 500 readings (~0.5 seconds) and 
/// setting that as the zero point for x and y and 9.81 m/s^2 for z.
/// 
/// The accelerometer should be as level as possible and not moving.
/// If the standard deviation is too high, the program will exit.
fn calibrate_accelerometer() {

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
