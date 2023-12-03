use std::time::Duration;
use adafruit_gps::{Gps, GpsSentence};
use adafruit_gps::NmeaOutput;

const PORT_NAME: &str  = "/dev/ttyS0";
const BAUD_RATE: &str = "9600";


fn main() {

  let mut gps = Gps::new(&PORT_NAME, &BAUD_RATE);
  init_gps(&mut gps);

  loop {
    let sentence = gps.update();
   
    // println!("{:?}", sentence); // print all data

    match sentence {
      GpsSentence::InvalidSentence => println!("-"),
      GpsSentence::InvalidBytes    => println!("-"),
      GpsSentence::NoConnection    => println!("Connecting..."),
      GpsSentence::GGA(sen) => {
        println!("Time: {}, Lat: {}, Lon: {}, Alt: {}, Hor_Prec: {}",
          sen.utc, sen.lat.unwrap_or(0.0), sen.long.unwrap_or(0.0), 
          sen.msl_alt.unwrap_or(0.0), sen.hdop.unwrap_or(0.0)); // change defaults to something that isn't possible
      },
      GpsSentence::GSA(sen) => {
        println!("Hor_Prec: {}, Ver_Prec: {}",
          sen.hdop.unwrap_or(0.0), sen.vdop.unwrap_or(0.0));
      },
      GpsSentence::GLL(sen) => {
        println!("Time: {}, Lat: {}, Lon: {}",
          sen.utc.unwrap_or(0.0), sen.latitude.unwrap_or(0.0), 
          sen.longitude.unwrap_or(0.0));
      },
      GpsSentence::RMC(sen) => {
        println!("Time: {}, Lat: {}, Lon: {}, Speed: {}, Date: {}",
          sen.utc, sen.latitude.unwrap_or(0.0), sen.longitude.unwrap_or(0.0),
          sen.speed.unwrap_or(-1.0), sen.date);
      },
      _ => {
        // ignore other sentence types (the data isn't as important for our purposes)
        ();
      }   
    }
  }
}

struct GpsCoord {
  lat: f32,
  lon: f32,
}

struct GpsData {
  lat: f32,   // rmc, 
  lon: f32,   // rmc, 
  alt: f32,   // gga
  speed: f32, // rmc, 
  time: f32,  // rmc, 
  date: &str, // rmc
  hor_prec: f32, // gsa, 
  ver_prec: f32, // gsa, 
}

/// setup gps refresh rate and which sentences to output
fn init_gps(gps: &mut Gps) {
  gps.pmtk_220_set_nmea_updaterate("1000"); // set update rate te 1 second
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

/// waits for gps to get a fix
fn wait_for_fix(gps: &mut Gps, mut timeout_sec: u32) -> Option<GpsCoord> {
  start = Instant::now();
  while timeout_sec > (Instant::now() - start) {
    let sentence = gps.update();
    if sentence == GpsSentence::GGA(sen) {
      if sen.sat_fix != NoFix {
        return (
          GpsCoord {
            lat: sen.lat.unwrap_or(0.0), // can I get rid of default values here?
            lon: sen.long.unwrap_or(0.0)
          }
        );
      }
    }
  }
  None // timed out before getting a fix
}

// get gps data or return none if no fix
fn get_gps() {
  todo!();
}


/// Computes the distance in meters between two points on earth.
/// Takes in latitude and longitude of two points, and returns 
/// the distance between them in meters.
fn haversine_distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    let r = 6371.0; // radius of earth in km
    let d_lat = degrees_to_radians(lat2 - lat1);
    let d_lon = degrees_to_radians(lon2 - lon1);
    let a = (d_lat/2.0).sin() * (d_lat/2.0).sin() + lat1.cos() * lat2.cos() * (d_lon/2.0).sin() * (d_lon/2.0).sin();
    let c = 2.0 * a.sqrt().atan2((1.0-a).sqrt());
    let d = r * c;
    d
}
