#include <TinyGPS++.h>
#include <math.h>
//#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
   
   Modified to connect to RX1 (pin 19) and TX1 (pin18) of Arduino Mega
   
*/
//static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

// variable for storing UTM data
double utm[3] = {0}; // [0] = x (UTM easting in meters), 
                    // [1] = y (UTM northing in meters, 
                    // [2] = zone (UTM longitudinal zone)

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);  
// SoftwareSerial did not work properly with shield and just caused problems in general


// convert longitude + latitude to UTM
// Originally from wgs2utm.m by Alexandre Schimel, MetOcean Solutions Ltd
// adapted from C++ code written by Raffi Zack
void wgs2utm(double latitude, double longitude)
{    
  //converting decimal degrees to UTM x,y
  double latitude_rad = latitude * M_PI/180; // latitude in rad
  double longitude_rad = longitude * M_PI/180; // longitude in rad
  
  //values
  double a = 6378137; // semi-major axis
  double b = 6356752.314245; // semi-minor axis
  double e = sqrt(1-pow((b/a),2)); // eccentricity
  double longitude_decimal0 = floor(longitude/6)*6+3; // ref longitude, deg
  double longitude_rad0 = longitude_decimal0 * M_PI/180; // ref longitude, rad
  double k0 = 0.9996; // central meridian scale
  double FE = 500000; // false easting
  double FN;
  if (latitude < 0) {
    FN = 10000000;
  }
  else {
    FN = 0; // false northing 
  }
  
  //equations
  double eps = pow(e,2)/(1-pow(e,2)); // e prime square
  double N = a/sqrt(1-pow(e,2)*pow(sin(latitude_rad),2)); // earth radius of curvature perpend. to meridian plane
  double T = pow(tan(latitude_rad),2);
  double C = ((pow(e,2))/(1-pow(e,2)))*pow(cos(latitude_rad),2);
  double A = (longitude_rad-longitude_rad0)*cos(latitude_rad);
  double M = a*((1-pow(e,2)/4 - 3*pow(e,4)/64 - 5*pow(e,6)/256)*latitude_rad - (3*pow(e,2)/8 + 3*pow(e,4)/32 + 45*pow(e,6)/1024)*sin(2*latitude_rad) + (15*pow(e,4)/256 + 45*pow(e,6)/1024)*sin(4*latitude_rad) - (35*pow(e,6)/3072)*sin(6*latitude_rad)); // true distance along the central meridian from equator to latitude
  double x = FE + k0*N*(A + (1-T+C)*pow(A,3)/6 + (5-18*T+pow(T,2)+72*C-58*eps)*pow(A,5)/120); // easting
  double y = FN + k0*M + k0*N*tan(latitude_rad)*(pow(A,2)/2 + (5-T+9*C+4*pow(C,2))*pow(A,4)/24 + (61-58*T+pow(T,2)+600*C-330*eps)*pow(A,6)/720); // northing

  utm[0] = x;
  utm[1] = y;
  
  //Serial.println("I got here to UTM!!!");
  
  
}//wgs2utm

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
/*
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
*/  
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      displayInfo();
      
      //Serial.println(gps.altitude.meters());
      
      wgs2utm(gps.location.lat(), gps.location.lng());
      Serial.print("UTM Easting = ");
      Serial.println(utm[0]);
      Serial.print("UTM Northing = ");
      Serial.println(utm[1]);
      
    }
    

}

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
