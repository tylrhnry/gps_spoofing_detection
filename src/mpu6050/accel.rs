use rppal::i2c::I2c;
use std::time::{Duration, Instant};
use std::f32::consts::PI;
use std::ops::{Add, Div};

const MPU6050_ADDR: u16 = 0x68;

fn main() {
  // Set up I2C device (the GY-521 accelerometer/gyro)
  let mut i2c = I2c::new().unwrap();
  i2c.set_slave_address(MPU6050_ADDR).unwrap();

  let _ = i2c.write_read(&[0x6B, 0x08], &mut [0; 1]);

  // Read accelerometer values
  let start = Instant::now();
  let mut iters = 0;

  let mut accel_vec: Vec<DataPoint> = Vec::new();
  let mut gyro_vec:  Vec<DataPoint> = Vec::new();

  loop {
    
    if start.elapsed() >= Duration::from_secs(1) {
      println!("Iterations in 1 sec.: {iters}");
      break;
    };

    let mut accel_data = [0; 6];
    let mut gyro_data  = [0; 6];
    let _ = i2c.write_read(&[0x3B], &mut accel_data);
    let _ = i2c.write_read(&[0x43], &mut gyro_data);
 

    let accel_x = i16::from_be_bytes([accel_data[0], accel_data[1]]);
    let accel_y = i16::from_be_bytes([accel_data[2], accel_data[3]]);
    let accel_z = i16::from_be_bytes([accel_data[4], accel_data[5]]);
    let accel_point = DataPoint {x: accel_x, y: accel_y, z: accel_z};
    accel_vec.push(accel_point);

    let gyro_x = i16::from_be_bytes([gyro_data[0], gyro_data[1]]);
    let gyro_y = i16::from_be_bytes([gyro_data[2], gyro_data[3]]);
    let gyro_z = i16::from_be_bytes([gyro_data[4], gyro_data[5]]);
    let gyro_point = DataPoint {x: gyro_x, y: gyro_y, z: gyro_z};
    gyro_vec.push(gyro_point);


    println!("Accelerometer: X={},\tY={},\tZ={}", accel_x, accel_y, accel_z);
    println!("Gyroscope:     X={},\tY={},\tZ={}", gyro_x, gyro_y, gyro_z);
    iters += 1;
  }

  let accel_avg = get_average(&accel_vec);
  let gyro_avg  = get_average(&gyro_vec);
  println!("Average accel: {:?}", accel_avg);
  println!("Average gyro:  {:?}", gyro_avg);
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



// impl pos_data {
//     fn new() -> pos_data { // modify this to take in the data from the GPS for starting pos.
//         pos_data {
//             pos_x: 0.0,
//             pos_y: 0.0,
//             pos_z: 0.0,
//             vel_x: 0.0,
//             vel_y: 0.0,
//             vel_z: 0.0,
//             acc_x: 0.0,
//             acc_y: 0.0,
//             acc_z: 0.0,
//             time: 0.0,
//         }
//     }
// }


/// calibrates the accelerometer at the start of the program
/// by taking the average of 500 readings (~0.5 seconds) and 
/// setting that as the zero point for x and y and 9.81 m/s^2 for z.
/// 
/// The accelerometer should be as level as possible and not moving.
/// If the standard deviation is too high, the program will exit.
fn calibrate_accelerometer() {
  todo!();
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


// cargo build
// Compiling acc_practice v0.1.0 (/home/tylrhnry/programming/rust/acc_practice)
// error[E0277]: cannot add `AccelPoint` to `AccelPoint`
// --> src/main.rs:53:47
// |
// 53 |   println!("Average accel: {:?}", get_average(&accel_vec));
// |                                   ----------- ^^^^^^^^^^ no implementation for `AccelPoint + AccelPoint`
// |                                   |
// |                                   required by a bound introduced by this call
// |
// = help: the trait `Add` is not implemented for `AccelPoint`
// note: required by a bound in `get_average`
// --> src/main.rs:72:26
// |
// 71 | fn get_average<T>(points: &[T]) -> T
// |    ----------- required by a bound in this function
// 72 |   where T: MotionPoint + Add<Output = T> + Div<i16, Output = T>
// |                          ^^^^^^^^^^^^^^^ required by this bound in `get_average`

// error[E0277]: cannot divide `AccelPoint` by `i16`
// --> src/main.rs:53:47
// |
// 53 |   println!("Average accel: {:?}", get_average(&accel_vec));
// |                                   ----------- ^^^^^^^^^^ no implementation for `AccelPoint / i16`
// |                                   |
// |                                   required by a bound introduced by this call
// |
// = help: the trait `Div<i16>` is not implemented for `AccelPoint`
// note: required by a bound in `get_average`
// --> src/main.rs:72:44
// |
// 71 | fn get_average<T>(points: &[T]) -> T
// |    ----------- required by a bound in this function
// 72 |   where T: MotionPoint + Add<Output = T> + Div<i16, Output = T>
// |                                            ^^^^^^^^^^^^^^^^^^^^ required by this bound in `get_average`

// error[E0277]: cannot add `GyroPoint` to `GyroPoint`
// --> src/main.rs:54:47
// |
// 54 |   println!("Average gyro:  {:?}", get_average(&gyro_vec));
// |                                   ----------- ^^^^^^^^^ no implementation for `GyroPoint + GyroPoint`
// |                                   |
// |                                   required by a bound introduced by this call
// |
// = help: the trait `Add` is not implemented for `GyroPoint`
// note: required by a bound in `get_average`
// --> src/main.rs:72:26
// |
// 71 | fn get_average<T>(points: &[T]) -> T
// |    ----------- required by a bound in this function
// 72 |   where T: MotionPoint + Add<Output = T> + Div<i16, Output = T>
// |                          ^^^^^^^^^^^^^^^ required by this bound in `get_average`

// error[E0277]: cannot divide `GyroPoint` by `i16`
// --> src/main.rs:54:47
// |
// 54 |   println!("Average gyro:  {:?}", get_average(&gyro_vec));
// |                                   ----------- ^^^^^^^^^ no implementation for `GyroPoint / i16`
// |                                   |
// |                                   required by a bound introduced by this call
// |
// = help: the trait `Div<i16>` is not implemented for `GyroPoint`
// note: required by a bound in `get_average`
// --> src/main.rs:72:44
// |
// 71 | fn get_average<T>(points: &[T]) -> T
// |    ----------- required by a bound in this function
// 72 |   where T: MotionPoint + Add<Output = T> + Div<i16, Output = T>
// |                                            ^^^^^^^^^^^^^^^^^^^^ required by this bound in `get_average`

// error[E0609]: no field `x` on type `&Self`
// --> src/main.rs:81:41
// |
// 79 | trait MotionPoint: Clone + Default {
// | ---------------------------------- type parameter 'Self' declared here
// 80 |   fn display(&self) {
// 81 |     println!("X={},\tY={},\tZ={}", self.x, self.y, self.z);
// |                                         ^

// error[E0609]: no field `y` on type `&Self`
// --> src/main.rs:81:49
// |
// 79 | trait MotionPoint: Clone + Default {
// | ---------------------------------- type parameter 'Self' declared here
// 80 |   fn display(&self) {
// 81 |     println!("X={},\tY={},\tZ={}", self.x, self.y, self.z);
// |                                                 ^

// error[E0609]: no field `z` on type `&Self`
// --> src/main.rs:81:57
// |
// 79 | trait MotionPoint: Clone + Default {
// | ---------------------------------- type parameter 'Self' declared here
// 80 |   fn display(&self) {
// 81 |     println!("X={},\tY={},\tZ={}", self.x, self.y, self.z);
// |                                                         ^

// Some errors have detailed explanations: E0277, E0609.
// For more information about an error, try `rustc --explain E0277`.
// error: could not compile `acc_practice` (bin "acc_practice") due to 7 previous errors
