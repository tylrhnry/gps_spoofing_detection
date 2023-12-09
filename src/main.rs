mod neo6m;
mod mpu6050;

use std::cell::{RefCell, RefMut};
use std::time::{ Instant, Duration };
use adafruit_gps::Gps;
use rppal::i2c::I2c;

const PORT_NAME: &str = "/dev/ttyS0";
const BAUD_RATE: &str = "9600";
const GPS_ACCURACY: f32 = 10.0; // meters
const GPS_FIX_TIMEOUT: Duration = Duration::from_secs(60); // How long to wait for gps fix before timing out


fn main() {
  // GPS setup
  #[allow(clippy::needless_borrow)] // In the future, PORT_NAME and BAUD_RATE will be used elsewhere
  let mut gps = Gps::new(&PORT_NAME, &BAUD_RATE);
  neo6m::gps::init_gps(&mut gps);

  // Accelerometer setup
  let i2c = RefCell::new(mpu6050::accel::init_mpu6050()); // Set up I2C device (the GY-521 accelerometer/gyro)

  println!("Calibrating MPU6050...");
  let (accel_offsets, _gyro_offsets) = {
    let i2c = i2c.borrow_mut();
    mpu6050::accel::calibrate_mpu6050(i2c, None, None, None) // Calibrate the accelerometer
  };
  println!("Calibration complete");


  println!("Waiting for gps fix...");
  let fix = neo6m::gps::wait_for_fix(&mut gps, GPS_FIX_TIMEOUT);
  if fix.is_none() {
    println!("Timed out waiting for gps fix");
    return; // change to waiting for gps fix again
  }

  let i2c = i2c.borrow_mut();
  detect_spoofing(&mut gps, &accel_offsets, &i2c);

}



/// Detects spoofing by comparing the predicted position to the actual position
fn detect_spoofing(gps: &mut Gps, accel_offsets: &mpu6050::accel::AccelPoint, i2c: &RefMut<'_, I2c>) {
  let mut gps_data = neo6m::gps::get_gps(gps).expect("Lost connection to gps");
  
  let mut x0 = neo6m::gps::GpsCoord::new(gps_data.lat(), gps_data.lon(), gps_data.alt()); // Initial position
  let mut v0 = mpu6050::accel::RawPoint::new(0.0, 0.0, 0.0); // Initial velocity

  loop {
    // predict position
    println!("using initial position of {x0}");
    let (predicted_pos, new_vel) = predict_position(500, i2c, accel_offsets, &x0, &v0);
    gps_data = neo6m::gps::get_gps(gps).expect("Lost connection to gps");
    x0 = neo6m::gps::GpsCoord::new(gps_data.lat(), gps_data.lon(), gps_data.alt()); // Initial position
    println!("{x0}");
    // compare
    let dist = neo6m::gps::haversine_distance(x0.lat(), x0.lon(), predicted_pos.lat(), predicted_pos.lon());
    let error_dist = gps_data.hor_prec() * GPS_ACCURACY;
    if dist > error_dist {
      println!("dist: {}, error_dist: {}", dist, error_dist);
      println!("Spoofing detected");
    }
    // reset values
    v0 = new_vel;
  }
}


/// Predicts the position of the gps using the accelerometer
fn predict_position(num_iters: u32, 
                    i2c: &RefMut<'_, I2c>, 
                    accel_offsets: &mpu6050::accel::AccelPoint, 
                    x0: &neo6m::gps::GpsCoord, 
                    v0: &mpu6050::accel::RawPoint) -> 
                      (neo6m::gps::GpsCoord, mpu6050::accel::RawPoint) {
  let start = Instant::now();
  let avg_accel = average_acceleration(num_iters, i2c, accel_offsets);
  let t = start.elapsed().as_secs_f64();

  let predicted_pos = neo6m::gps::calc_new_pos(x0, v0, &avg_accel, &t);
  let v0 = neo6m::gps::calc_new_vel(v0, &avg_accel, &t);

  (predicted_pos, v0)
}


/// Returns the average acceleration over a period of time
fn average_acceleration(num_iters: u32, i2c: &RefMut<'_, I2c>, accel_offsets: &mpu6050::accel::AccelPoint) -> mpu6050::accel::RawPoint {
  let mut accel_sum = mpu6050::accel::RawPoint::new(0.0, 0.0, 0.0);
  for _ in 0..num_iters {
    let accel_point = mpu6050::accel::get_converted_acceleration(i2c, accel_offsets);
    accel_sum += accel_point;
  }
  accel_sum / num_iters as f32
}

