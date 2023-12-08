mod gps;
mod mpu6050;

use std::cell::{RefCell, RefMut};
use std::borrow::BorrowMut;
use std::time::{ Instant, Duration };
use adafruit_gps::Gps;
use rppal::i2c::I2c;

const PORT_NAME: &str = "/dev/ttyS0";
const BAUD_RATE: &str = "9600";
const GPS_ACCURACY_THRESHOLD: f32 = 10.0; // meters
const GPS_FIX_TIMEOUT: Duration = Duration::from_secs(60); // How long to wait for gps fix before timing out


fn main() {
  // GPS setup
  let mut gps = Gps::new(&PORT_NAME, &BAUD_RATE);
  gps::gps::init_gps(&mut gps);

  // Accelerometer setup
  let i2c = RefCell::new(mpu6050::accel::init_mpu6050()); // Set up I2C device (the GY-521 accelerometer/gyro)

  println!("Calibrating MPU6050...");
  let (accel_offsets, gyro_offsets) = {
    let i2c = i2c.borrow_mut();
    mpu6050::accel::calibrate_mpu6050(i2c, None, None, None) // Calibrate the accelerometer
  };
  println!("Calibration complete");


  println!("Waiting for gps fix...");
  let fix = gps::gps::wait_for_fix(&mut gps, GPS_FIX_TIMEOUT);
  if fix.is_none() {
    println!("Timed out waiting for gps fix");
  }


  loop {
    let gps_data = gps::gps::get_gps(&mut gps).expect("Lost connection to gps");
    println!("{:?}", gps_data);

    for _ in 0..500 {
      let accel_point = {
        let i2c = i2c.borrow_mut();
        mpu6050::accel::get_converted_acceleration(&i2c, &accel_offsets)
      };
      let gyro_point = {
        let i2c = i2c.borrow_mut();
        mpu6050::accel::get_offset_gyroscope(&i2c, &gyro_offsets)
      };
      print!("{accel_point}\t");
      println!("{gyro_point}");
    }
  }
}

#[allow(dead_code)]
fn detect_spoofing(accel_offsets: &mpu6050::accel::AccelPoint, i2c: &RefMut<'_, I2c>) {
  let t0 = Instant::now();
  let x0 = gps::gps::GpsCoord::new(0.0, 0.0, 0.0); // Initial position
  let v0 = mpu6050::accel::RawPoint::new(0.0, 0.0, 0.0); // Initial velocity

  loop {
    let accel_point = mpu6050::accel::get_converted_acceleration(i2c, accel_offsets);
    let t = t0.elapsed().as_secs_f64();
    
    let new_pos = gps::gps::calc_new_pos(&x0, &v0, &accel_point, &t);
    let new_vel = gps::gps::calc_new_vel(&v0, &accel_point, &t);
    // if new_pos.distance_to(&last_pos) > GPS_ACCURACY_THRESHOLD {
    //   println!("Spoofing detected");
    // }
  }
}




