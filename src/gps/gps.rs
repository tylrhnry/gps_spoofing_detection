use std::time::Duration;
use adafruit_gps::{Gps, GpsSentence};
use adafruit_gps::NmeaOutput;


fn main() {
    
  let port_name = "/dev/ttyS0";
  let baud_rate = "9600";

  let mut gps = Gps::new(&port_name, &baud_rate);
  gps.pmtk_220_set_nmea_updaterate("1000"); // set update rate te 1 second
  gps.pmtk_314_api_set_nmea_output(NmeaOutput{gga: 1, gsa: 1, gsv: 1, gll: 1, // modify to only get the data we need
                                              rmc: 1, vtg: 1, pmtkchn_interval: 1});

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
