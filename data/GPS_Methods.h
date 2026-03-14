/*
  All GPS related methods are here
*/


#include <TinyGPS++.h>  // get library here > http://arduiniana.org/libraries/tinygpsplus/
// https://github.com/mikalhart/TinyGPSPlus version 1.0.3
TinyGPSPlus gps;  // create the TinyGPS++ object
// Create an instance of the HardwareSerial class for Serial 1 for GPS

#define NEO7M_GPS_RX_PIN 7   // The GPS board has an RX terminal - connect it with GPIO 7
#define NEO7M_GPS_TX_PIN 6   // The GPS board has a TX terminal - connect it with GPIO 6
#define HARDWARE_SERIAL_PORT 1 // on ESP32 modules you should use port 2

HardwareSerial gpsSerial(HARDWARE_SERIAL_PORT);

bool isGpsActivated = false;
bool isWaitingForFirstGpsFix = false;
bool GPSfix = false;
// https://kleuthold.wordpress.com/2019/09/05/gps-modul-neo-6m-im-einsatz/
float TXLat;      // Latitude from GPS on Tracker transmitter (TX)
float TXLon;      // Longitude from GPS on Tracker transmitter (TX)
float TXAlt;      // Altitude from GPS on Tracker transmitter (TX)
uint8_t TXSats;   // number of GPS satellites seen (TX)
uint32_t TXHdop;  // HDOP from GPS on Tracker transmitter (TX), horizontal dilution of precision
// https://novotech.com/pages/horizontal-dilution-of-precision-hdop
float TXHdopFloat;      // HDOP from GPS on Tracker transmitter (TX)
uint32_t TXGPSFixTime;  // GPS fix time in hot fix mode of GPS on Tracker transmitter (TX)

// is the gps data updated by a new fix ?
uint8_t TXGpsUpdateStatus = 0;
bool TXGpsLocationIsUpdated;    // TXGpsUpdateStatus Bit 0
bool TXGpsDateIsUpdated;        // TXGpsUpdateStatus Bit 1
bool TXGpsTimeIsUpdated;        // TXGpsUpdateStatus Bit 2
bool TXGpsSpeedIsUpdated;       // TXGpsUpdateStatus Bit 3
bool TXGpsCourseIsUpdated;      // TXGpsUpdateStatus Bit 4
bool TXGpsAltitudeIsUpdated;    // TXGpsUpdateStatus Bit 5
bool TXGpsSatellitesIsUpdated;  // TXGpsUpdateStatus Bit 6
bool TXGpsHdopIsUpdated;        // TXGpsUpdateStatus Bit 7

uint32_t TXGpsLocationAge;
bool TXGpsDateIsValid;

bool TXGpsTimeIsValid;
time_t localTime;
uint16_t TXyear;
uint8_t TXmonth;
uint8_t TXday;
uint8_t TXhour;
uint8_t TXminute;
uint8_t TXsecond;
uint8_t TXGpsDateIsValid8;
uint8_t TXGpsDateIsUpdated8;
uint8_t TXGpsTimeIsValid8;
uint8_t TXGpsTimeIsUpdated8;
double TXGpsSpeedKmh;
double TXGpsCourseDegree;
float TXGpsDistanceToReference;
uint16_t TXGpsCourseToReference;

uint32_t loopCnt = 0;

#define GPS_DATA_INTERVAL_SECONDS 10  // the GPS data were retrieved every xx interval seconds
#define WaitGPSFixSeconds 10          // time in seconds to wait for a new GPS fix
#define WaitFirstGPSFixSeconds 120    // time to seconds to wait for the first GPS fix at startup

const unsigned long GPS_DATA_INTERVAL_MILLIS = GPS_DATA_INTERVAL_SECONDS * 1000;
long lastGpsDataMillis = 0;

#define NEO7M_GPS_BAUD 9600  // The GPS Baud rate on hardware serial

// taken from Google Maps: Rhine Tower in Duesseldorf, Northrhine Westfalia, Germany
// 51.21800467274397, 6.761672316462719
#define GPS_REFERENCE_LAT 51.21800467274397
#define GPS_REFERENCE_LON 6.761672316462719

//#define GPS_TEST_MODE 1
// A sample NMEA stream for testing.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// -----------------------------------------------------------------------
// UTC time converting
#include <Timezone.h>  // https://github.com/JChristensen/Timezone

// see here: https://github.com/JChristensen/Timezone/blob/master/examples/WorldClock/WorldClock.ino
// Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };  // Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 60 };    // Central European Standard Time
Timezone CE(CEST, CET);

TimeChangeRule *tcr;  // pointer to the time change rule, use to get TZ abbrev

void displayGpsData() {
  Serial.printf("Current location Lat: %s Lon: %s Alt: %s\n", String(TXLat, 4), String(TXLon, 4), String(TXAlt, 1));
}

// Does the GPS module respond ?
bool GPSTest() {
  bool success = false;
  uint32_t counter = 0;
  uint32_t startmS;
  startmS = millis();
  while ((uint32_t)(millis() - startmS) < 2000)  // allows for millis() overflow
  {
    if (gpsSerial.available() > 0) {
      Serial.write(gpsSerial.read());
      counter++;
      if (counter > 3) success = true;
    }
  }
  Serial.println();
  Serial.flush();
  return success;
}

void setTXGpsStatusByte(uint8_t bitnum, uint8_t bitval) {
  // program the status byte
  if (bitval == 0) {
    bitClear(TXGpsUpdateStatus, bitnum);
  } else {
    bitSet(TXGpsUpdateStatus, bitnum);
  }
}

// taken from https://github.com/StuartsProjects/SX12XX-LoRa/blob/bc9cf344c07c79dc1486514a35f36919e331831e/examples/SX126x_examples/Tracker/23_GPS_Tracker_Transmitter/23_GPS_Tracker_Transmitter.ino
bool gpsWaitFix(uint32_t waitSecs) {
  //waits a specified number of seconds for a fix, returns true for good fix
  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F("s"));

  uint32_t endwaitmS, GPSonTime;
  bool GPSFix = false;
  float tempfloat;
  uint8_t GPSchar;

  TXGpsUpdateStatus = 0;
  TXGpsLocationIsUpdated = false;
  TXGpsDateIsUpdated = false;
  TXGpsTimeIsUpdated = false;
  TXGpsSpeedIsUpdated = false;
  TXGpsCourseIsUpdated = false;
  TXGpsAltitudeIsUpdated = false;
  TXGpsSatellitesIsUpdated = false;
  TXGpsHdopIsUpdated = false;

  GPSonTime = millis();
  gpsSerial.begin(NEO7M_GPS_BAUD, SERIAL_8N1, NEO7M_GPS_TX_PIN, NEO7M_GPS_RX_PIN);  // start GPSserial

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS) {
    if (gpsSerial.available() > 0) {
      GPSchar = gpsSerial.read();
      gps.encode(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
      GPSfix = true;
      Serial.print(F("Have GPS Fix "));
      TXGPSFixTime = millis() - GPSonTime;
#ifdef DEBUG_PRINT_GPS_DATA
      Serial.print(TXGPSFixTime);
      Serial.println(F(" mS"));
#endif      
      break;  //exit while loop reading GPS
    }
  }

  if (gps.location.isUpdated()) {
    TXGpsLocationIsUpdated = true;
    TXLat = gps.location.lat();
    TXLon = gps.location.lng();
    TXGpsLocationAge = gps.location.age();
    setTXGpsStatusByte(0, 1);

    // calculate the distance and course to the reference GPS coordinates
    TXGpsDistanceToReference = TinyGPSPlus::distanceBetween(TXLat, TXLon, GPS_REFERENCE_LAT, GPS_REFERENCE_LON);
    TXGpsCourseToReference = (int16_t)TinyGPSPlus::courseTo(TXLat, TXLon, GPS_REFERENCE_LAT, GPS_REFERENCE_LON);
  }

  if (gps.date.isUpdated()) {
    TXGpsDateIsUpdated = true;
    TinyGPSDate dt = gps.date;
    TXyear = dt.year();
    TXmonth = dt.month();
    TXday = dt.day();
    setTXGpsStatusByte(1, 1);
  }

  if (gps.time.isUpdated()) {
    TXGpsTimeIsUpdated = true;
    TinyGPSTime tm = gps.time;
    TXhour = tm.hour();
    TXminute = tm.minute();
    TXsecond = tm.second();
    setTXGpsStatusByte(2, 1);
  }

  if (gps.speed.isUpdated()) {
    TXGpsSpeedIsUpdated = true;
    TXGpsSpeedKmh = gps.speed.kmph();
    setTXGpsStatusByte(3, 1);
  }

  if (gps.course.isUpdated()) {
    TXGpsCourseIsUpdated = true;
    TXGpsCourseDegree = gps.course.deg();
    setTXGpsStatusByte(4, 1);
  }

  if (gps.altitude.isUpdated()) {
    TXGpsAltitudeIsUpdated = true;
    TXAlt = gps.altitude.meters();
    setTXGpsStatusByte(5, 1);
  }

  if (gps.satellites.isUpdated()) {
    TXGpsSatellitesIsUpdated = true;
    TXSats = gps.satellites.value();
    setTXGpsStatusByte(6, 1);
  }

  if (gps.hdop.isUpdated()) {
    TXGpsHdopIsUpdated = true;
    TXHdop = gps.hdop.value();
    TXHdopFloat = ((float)TXHdop / 100.0);
    setTXGpsStatusByte(7, 1);
  }

  Serial.print(TXLat, 5);
  Serial.print(F(","));
  Serial.print(TXLon, 5);
  Serial.print(F(","));
  Serial.print(TXAlt, 1);
  Serial.print(F(","));
  Serial.print(TXSats);
  Serial.print(F(","));
  Serial.print(tempfloat, 2);
  Serial.println();

  // this is the time from the satellite
#ifdef DEBUG_PRINT_GPS_DATA  
  char bufTime[60];
  sprintf(bufTime, "h: %d m: %d s: %d dy: %d mn: %d yr: %d GMT\n", TXhour, TXminute, TXsecond, TXday, TXmonth, TXyear);
  Serial.print(bufTime);
#endif

  // conversion utc/gps time to local time
  //setTime(myTZ.toUTC(compileTime()));
  //setTime(01, 55, 00, 11, 3, 2012); //another way to set the time (hr,min,sec,day,mnth,yr)
  setTime(TXhour, TXminute, TXsecond, TXday, TXmonth, TXyear);  // another way to set the time (hr,min,sec,day,mnth,yr)
  isTimeAvailable = true;

#ifdef DEBUG_PRINT_GPS_DATA
  // time from Timezone library
  time_t utc = now();
  localTime = CE.toLocal(utc, &tcr);
  sprintf(bufTime, "h: %d m: %d s: %d dy: %d mn: %d yr: %d %s\n", hour(localTime), minute(localTime), second(localTime), day(localTime), month(localTime), year(localTime), tcr->abbrev);
  Serial.print(bufTime);
#endif

  //if here then there has either been a fix or no fix and a timeout

  if (GPSfix) {
    // GPSfix is already set to true
  } else {
    Serial.println();
    Serial.println(F("Timeout - No GPSFix"));
    Serial.println();
    GPSfix = false;
  }

  gpsSerial.end();  //serial RX interrupts interfere with SPI, so stop GPSserial
  return GPSfix;
}

void printGpsData() {
  Serial.println();
  Serial.println(F("--== GPS Data ==--"));
  Serial.print(F("LAT: "));
  Serial.print(TXLat, 8);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("LOT:"));
  Serial.print(TXLon, 8);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("Location Age: "));
  Serial.print(TXGpsLocationAge);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("Alt:"));
  Serial.print(TXAlt);
  Serial.print(F(" m"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsAltitudeIsUpdated);
  char buf[32];
  char m[4];  // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(localTime)));
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
          hour(localTime), minute(localTime), second(localTime), dayShortStr(weekday(localTime)), day(localTime), m, year(localTime) - 2000, tcr->abbrev);
  Serial.print(F("Date & Time local: "));
  Serial.print(buf);
  Serial.print(F(""));
  Serial.print(F(" upd Date: "));
  Serial.print(TXGpsDateIsUpdated);
  Serial.print(F(" upd Time: "));
  Serial.println(TXGpsTimeIsUpdated);
  Serial.print(F("Speed: "));
  Serial.print(TXGpsSpeedKmh);
  Serial.print(F(" km/h"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsSpeedIsUpdated);
  Serial.print(F("Course: "));
  Serial.print(TXGpsCourseDegree);
  Serial.print(F(" degrees"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsCourseIsUpdated);
  Serial.print(F("Sats: "));
  Serial.print(TXSats);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsSatellitesIsUpdated);
  Serial.print(F("Horizontal dilution of precision (HDOP): "));
  Serial.print(TXHdop);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsHdopIsUpdated);
  Serial.print(F("Distance to reference: "));
  Serial.print(TXGpsDistanceToReference);
  Serial.print(F(" m"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);

  const char *cardinalToReference = TinyGPSPlus::cardinal(TXGpsCourseToReference);
  Serial.print(F("CourseToReference: "));
  Serial.print(TXGpsCourseToReference);
  Serial.print(F(" degrees"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("CourseToReference: "));
  Serial.print(cardinalToReference);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
}
