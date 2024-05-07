// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
// Connect TMP36 signal (middle pin) to Analog 0
// Connect LSM303 SCL to SCL (Analog 5)
// Connect LSM303 SCA to SCA (Analog 4)

#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

bool hasPrintedInitialInfo = false;
unsigned int TMP36_DATA_PIN = 0;
unsigned int DATA_DELAY_MILLIS = 1000U * 5;
float KNOTS_TO_METRES_PER_SECOND = 1.9438444924;
String DASHES = "------------------------";

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void printWithLeadingZero(int val) {
  if (val < 10) {
    Serial.print('0');
  }
  Serial.print(val);
}

void printGPSDateTime() {
  Serial.print(2000 + GPS.year); Serial.print('-');
  printWithLeadingZero(GPS.month); Serial.print('-');
  printWithLeadingZero(GPS.day); Serial.print('T');
  printWithLeadingZero(GPS.hour); Serial.print(':');
  printWithLeadingZero(GPS.minute); Serial.print(':');
  printWithLeadingZero(GPS.seconds); Serial.print('.');

  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print('0');
  }
  Serial.print(GPS.milliseconds);
  Serial.println('Z');
}

void printGPSFixInfo() {
  if (GPS.fix) {
    if (hasPrintedInitialInfo) {
      return;
    }

    // Print the ancillary GPS info once after we get a fix, then stop doing so
    hasPrintedInitialInfo = true;
    Serial.println("GPS fix received.");
  } else {
    Serial.println("No GPS fix yet");
  }
  
  Serial.print("-- Fix: "); Serial.println(GPS.fix);
  Serial.print("-- Fix quality: "); Serial.println((int)GPS.fixquality);
  Serial.print("-- Satellites: "); Serial.println((int)GPS.satellites);

  Serial.print("-- Antenna status: ");
  int antennaStatus = (int)GPS.antenna;
  if (antennaStatus == 1){
    Serial.println("ERROR");
  } else if (antennaStatus == 2) {
    Serial.println("Internal");
  } else if (antennaStatus == 3) {
    Serial.println("External");
  } else {
    Serial.println("UNKNOWN");
  }
}

void printGPSLocationInfo() {
  if (!GPS.fix) return;
  
  float lat = GPS.latitude;
  float lon = GPS.longitude;
  int latDegrees = int(lat / 100);
  float latMinutes = lat - (latDegrees * 100);
  int lonDegrees = int(lon / 100);
  float lonMinutes = lon - (lonDegrees * 100);
  Serial.print("Latitude: "), Serial.print(latDegrees); Serial.print("º"); Serial.print(latMinutes, 4); Serial.println(GPS.lat);
  Serial.print("Longitude: "), Serial.print(lonDegrees); Serial.print("º"); Serial.print(lonMinutes, 4); Serial.println(GPS.lon);
  Serial.print("Speed: "); Serial.print(GPS.speed * KNOTS_TO_METRES_PER_SECOND); Serial.println("m/s");
  Serial.print("Heading: "); Serial.print(GPS.angle); Serial.print("º"); Serial.println(GPS.mag);
  Serial.print("Altitude: "); Serial.print(GPS.altitude); Serial.println("m");
}

void printTemperatureInfo() {
  // Temperature = 10mv/ºC, with a 500mV offset
  // Also converting from 0 to 1024 digital range to 0-5V
  float temperature = (analogRead(TMP36_DATA_PIN) * 0.004882814 - 0.500) * 100;
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println("ºC");
}

void printAccelerometerInfo() {
  sensors_event_t event;
  accel.getEvent(&event);

  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(' ');
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(' ');
  Serial.print("Z: "); Serial.print(event.acceleration.z);
  Serial.println(" (m/s^2)");
}

void setup() {
  // Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(DASHES);
  Serial.println("Data logger start.");
  Serial.println(DASHES);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // GPS setup magic. See the library docs for explanations here, it's way over my head
  // (this all just came from the library example)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  if (!accel.begin()) {
    Serial.println("No LSM303 detected");
    while (1);
  } else {
    Serial.println("LSM303 initialized");
    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_LOW_POWER);
  }

  delay(2000);
}

uint32_t timer = millis();
void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  if (millis() - timer > DATA_DELAY_MILLIS) {
    timer = millis();
    Serial.println(DASHES);
    printGPSDateTime();
    printGPSFixInfo();
    printGPSLocationInfo();
    printTemperatureInfo();
    printAccelerometerInfo();
  }
}
