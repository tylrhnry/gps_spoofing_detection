use adafruit_gps::gga::SatFix::NoFix;
use adafruit_gps::NmeaOutput;
use adafruit_gps::{Gps, GpsSentence};
use std::f32::consts::PI;
use std::time::{Instant, Duration};
use std::fmt::Display;

use crate::mpu6050::accel::RawPoint;


// const PORT_NAME: &str = "/dev/ttyS0";
// const BAUD_RATE: &str = "9600";
// const GPS_FIX_TIMEOUT: Duration = Duration::from_secs(60); // How long to wait for gps fix before timing out
const UPDATE_RATE: &str = "1000"; // How often to update gps in milliseconds. Baud rate must change with this
const EARTH_RAD: f32 = 6371000.0; // radius of earth in km


/// Struct to hold gps coordinates
pub struct GpsCoord {
  lat: f32,
  lon: f32,
  alt: f32,
}

#[derive(Debug, Default)]
pub struct GpsData {
  lat: f32,   // rmc, 
  lon: f32,   // rmc, 
  alt: f32,   // gga
  speed: f32, // rmc, 
  time: f64,  // rmc, 
  date: String, // rmc
  hor_prec: f32, // gsa, 
  ver_prec: f32, // gsa, 
}


/// setup gps refresh rate and which sentences to output
pub fn init_gps(gps: &mut Gps) {
  gps.pmtk_220_set_nmea_updaterate(UPDATE_RATE); // set update rate te 1 second
  gps.pmtk_314_api_set_nmea_output(
    // every _ updates, give me the sentence (1 = every update, 0 = never)
    NmeaOutput{
      // the current selection of data to extract includes all the 
      // information to instantiate a GpsData struct
      gga: 1, 
      gsa: 1, 
      gsv: 0, 
      gll: 0, 
      rmc: 1, 
      vtg: 0, 
      pmtkchn_interval: 0
    }
  );
}

/// waits for gps to get a fix or times out
pub fn wait_for_fix(gps: &mut Gps, timeout_sec: Duration) -> Option<GpsCoord> {
  let start = Instant::now();
  while timeout_sec > (Instant::now() - start) {
    let sentence = gps.update();
    if let GpsSentence::GGA(sen) = sentence {
      if sen.sat_fix != NoFix {
        return Some(
          GpsCoord {
            lat: sen.lat.unwrap_or(0.0), // can I get rid of default values here?
            lon: sen.long.unwrap_or(0.0),
            alt: sen.msl_alt.unwrap_or(0.0),
          }
        );
      }
    }
  }
  None // timed out before getting a fix
}

/// get gps data or return none if no fix
pub fn get_gps(gps: &mut Gps) -> Option<GpsData> {
  let mut data = GpsData::new();

  let mut rmc = false;
  let mut gga = false;
  let mut gsa = false;

  while !rmc || !gga || !gsa {
    let sentence = gps.update();
    match sentence {
      GpsSentence::InvalidSentence => return None, // Invalid checksum
      GpsSentence::InvalidBytes    => return None, // Port and gps baud rate don't match
      GpsSentence::NoConnection    => return None, // Gps not connected, not receiving bytes
      GpsSentence::RMC(sen) => {
        data.lat = sen.latitude.unwrap_or(0.0);
        data.lon = sen.longitude.unwrap_or(0.0);
        data.speed = sen.speed.unwrap_or(-1.0);
        data.time = sen.utc;
        data.date = sen.date;
        rmc = true;
      }
      GpsSentence::GGA(sen) => {
        data.alt = sen.msl_alt.unwrap_or(-1.0);
        gga = true;
      }
      GpsSentence::GSA(sen) => {
        data.hor_prec = sen.hdop.unwrap_or(-1.0);
        data.ver_prec = sen.vdop.unwrap_or(-1.0);
        gsa = true;
      }
      _ => {
        // ignore other sentence types (the data isn't as important for our purposes)
      }   
    }
  }
  Some(data)
}


/// Predicts the position of the gps by averaging the acceleration over a period of time
pub fn calc_new_pos(old_pos: &GpsCoord, vel: &RawPoint, accel: &RawPoint, time: &f64) -> GpsCoord {
  let new_lat = old_pos.lat + vel.x() * (*time as f32) + 0.5 * accel.x() * (*time as f32).powi(2);
  let new_lon = old_pos.lon + vel.y() * (*time as f32) + 0.5 * accel.y() * (*time as f32).powi(2);
  let new_alt = old_pos.alt + vel.z() * (*time as f32) + 0.5 * accel.z() * (*time as f32).powi(2);
  GpsCoord::new(new_lat, new_lon, new_alt)
}


/// Predicts the velocity of the gps by averaging the acceleration over a period of time
pub fn calc_new_vel(old_vel: &RawPoint, accel: &RawPoint, time: &f64) -> RawPoint {
  let new_lat = old_vel.x() + accel.x() * (*time as f32);
  let new_lon = old_vel.y() + accel.y() * (*time as f32);
  let new_alt = old_vel.z() + accel.z() * (*time as f32);
  RawPoint::new(new_lat, new_lon, new_alt)
}


fn degrees_to_radians(degrees: f32) -> f32 {
  degrees * PI / 180.0
}

/// Computes the distance in meters between two points on earth.
/// Takes in latitude and longitude of two points, and returns
/// the distance between them in meters.
pub fn haversine_distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
  let d_lat = degrees_to_radians(lat2 - lat1);
  let d_lon = degrees_to_radians(lon2 - lon1);

  let lat1 = degrees_to_radians(lat1);
  let lat2 = degrees_to_radians(lat2);
  
  let a = ((d_lat / 2.0).sin()).powi(2) +
               ((d_lon / 2.0).sin()).powi(2) * lat1.cos() * lat2.cos();
  let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
  c * EARTH_RAD
}


/// GpsCoord implementations
impl GpsCoord {
  pub fn new(lat: f32, lon: f32, alt: f32) -> GpsCoord {
    GpsCoord { lat, lon, alt }
  }

  pub fn lat(&self) -> f32 {
    self.lat
  }

  pub fn lon(&self) -> f32 {
    self.lon
  }

  #[allow(dead_code)] // altitude will be taken into account later
  pub fn alt(&self) -> f32 {
    self.alt
  }
}

impl Display for GpsCoord {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "(lat: {}, lon: {}, alt: {})", self.lat, self.lon, self.alt)
  }
}


/// GpsData implementations
impl GpsData {
  pub fn new() -> GpsData {
    GpsData {
      lat: 0.0,
      lon: 0.0,
      alt: 0.0,
      speed: 0.0,
      time: 0.0,
      date: String::new(),
      hor_prec: 0.0,
      ver_prec: 0.0,
    }
  }

  #[allow(dead_code)]
  pub fn lat(&self) -> f32 {
    self.lat
  }

  #[allow(dead_code)]
  pub fn lon(&self) -> f32 {
    self.lon
  }

  #[allow(dead_code)]
  pub fn alt(&self) -> f32 {
    self.alt
  }

  #[allow(dead_code)]
  pub fn speed(&self) -> f32 {
    self.speed
  }

  #[allow(dead_code)]
  pub fn time(&self) -> f64 {
    self.time
  }

  #[allow(dead_code)]
  pub fn date(&self) -> &String {
    &self.date
  }

  pub fn hor_prec(&self) -> f32 {
    self.hor_prec
  }

  #[allow(dead_code)]
  pub fn ver_prec(&self) -> f32 {
    self.ver_prec
  }
}



#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_calc_new_pos() {
    let old_pos = GpsCoord::new(0.0, 0.0, 0.0);
    let vel = RawPoint::new(1.0, 1.0, 1.0);
    let accel = RawPoint::new(1.0, 1.0, 1.0);
    let time = 1.0;
    let new_pos = calc_new_pos(&old_pos, &vel, &accel, &time);
    assert_eq!(new_pos.lat(), 1.5);
    assert_eq!(new_pos.lon(), 1.5);
    assert_eq!(new_pos.alt(), 1.5);
  }

  #[test]
  fn test_calc_new_vel() {
    let old_vel = RawPoint::new(0.0, 0.0, 0.0);
    let accel = RawPoint::new(1.0, 1.0, 1.0);
    let time = 1.0;
    let new_vel = calc_new_vel(&old_vel, &accel, &time);
    assert_eq!(new_vel.x(), 1.0);
    assert_eq!(new_vel.y(), 1.0);
    assert_eq!(new_vel.z(), 1.0);
  }

  #[test]
  fn test_degrees_to_radians() {
    let deg = 180.0;
    let rad = degrees_to_radians(deg);
    assert_eq!(rad, PI);
  }

  #[test]
  fn test_haversine_distance() {
    let lat1 = 0.0;
    let lon1 = 0.0;
    let lat2 = 0.0;
    let lon2 = 0.0;
    let dist = haversine_distance(lat1, lon1, lat2, lon2);
    assert_eq!(dist, 0.0);

    let lat1 = 0.0;
    let lon1 = 0.0;
    let lat2 = 10.0;
    let lon2 = 10.0;
    let dist = haversine_distance(lat1, lon1, lat2, lon2);
    assert_eq!(dist, 1568520.6);
  }
}