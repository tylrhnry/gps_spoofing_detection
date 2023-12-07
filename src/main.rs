use project_root::accel::accel::init_accel;
use project_root::gps::gps::init_gps;


use std::time::Instant;

const GPS_ACCURACY_THRESHOLD: f32 = 10.0; // meters

fn main() {
  // GPS setup
  let mut gps = Gps::new(&PORT_NAME, &BAUD_RATE);
  init_gps(&mut gps);

  // Accelerometer setup
  let i2c = RefCell::new(init_mpu6050()); // Set up I2C device (the GY-521 accelerometer/gyro)
  let i2c = i2c.borrow_mut();

  println!("Calibrating MPU6050...");
  let (accel_offsets, gyro_offsets) = calibrate_mpu6050(&i2c, None, None, None); // Calibrate the accelerometer
  println!("Calibration complete");


  println!("Waiting for gps fix...");
  let fix = wait_for_fix(&mut gps, GPS_FIX_TIMEOUT);
  if let None = fix {
    println!("Timed out waiting for gps fix");
  }


  loop {
    let gps_data = get_gps(&mut gps).expect("Lost connection to gps");
    println!("{:?}", gps_data);


    let accel_point = get_converted_acceleration(&i2c, &accel_offsets);
    let gyro_point = get_offset_gyroscope(&i2c, &gyro_offsets);
    print!("{accel_point}\t");
    println!("{gyro_point}");
  }
}


fn detect_spoofing() {
  let t0 = Instant::now();
  let x0 = DataPoint {0, 0, 0}; // Initial position
  let v0 = 0.0; // Initial velocity

  loop {
    let accel_point = get_converted_acceleration(&i2c, accel_offsets);
    let t = t0.elapsed().as_secs_f64();
    
    let x_x = x0 + v0 * t + 0.5 * accel_point.x * t.powi(2);
    let x_y = x0 + v0 * t + 0.5 * accel_point.y * t.powi(2);
    let x_z = x0 + v0 * t + 0.5 * accel_point.z * t.powi(2);
    
    let v_x = v0 + accel_point.x * t;
    let v_y = v0 + accel_point.y * t;
    let v_z = v0 + accel_point.z * t;

    let new_pos = GpsCoord {lat: x_x, lon: x_y, alt: x_z};
    if new_pos.distance_to(&last_pos) > GPS_ACCURACY_THRESHOLD {
      println!("Spoofing detected");
    }
  }
}




