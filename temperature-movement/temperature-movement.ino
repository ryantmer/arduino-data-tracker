// Connect TMP36 signal (middle pin) to Analog 0
// Connect LSM303 SCL to SCL (Analog 5)
// Connect LSM303 SCA to SCA (Analog 4)

#include "RTClib.h"
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

unsigned const int TMP36_DATA_PIN = 0;
unsigned const int CHIP_SELECT_PIN = 10;
unsigned const int PERIOD_SECONDS = 5;
float PERIOD_MAX_X = 0.0;
float PERIOD_MAX_Y = 0.0;
float PERIOD_MAX_Z = 0.0;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
RTC_DS1307 rtc;

void setup() {
  Serial.begin(9600);
  
  Serial.println("Data logger start.");

  if (!rtc.begin()) {
    Serial.println("No RTC detected");
    while (1);
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC not running, setting time to now");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD card failed, or not present");
    while (1);
  }

  if (!accel.begin()) {
    Serial.println("No LSM303 detected");
    while (1);
  } else {
    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_LOW_POWER);
  }

  Serial.println("Setup complete, begin logging data...");

  delay(2000);
}

uint32_t timer = millis();
void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  if (abs(event.acceleration.x) > abs(PERIOD_MAX_X)) {
    PERIOD_MAX_X = event.acceleration.x;
  }
  if (abs(event.acceleration.y) > abs(PERIOD_MAX_Y)) {
    PERIOD_MAX_Y = event.acceleration.y;
  }
  if (abs(event.acceleration.z) > abs(PERIOD_MAX_Z)) {
    PERIOD_MAX_Z = event.acceleration.z;
  }

  if (millis() - timer > PERIOD_SECONDS * 1000) {
    DateTime now = rtc.now();
  
    // Temperature = 10mv/ºC, with a 500mV offset
    // Also converting from 0 to 1024 digital range to 0-5V
    float temperature = (analogRead(TMP36_DATA_PIN) * 0.004882814 - 0.500) * 100;
  
    String datum =
      now.timestamp() + ',' +
      String(temperature) + ',' +
      String(PERIOD_MAX_X) + ',' +
      String(PERIOD_MAX_Y) + ',' +
      String(PERIOD_MAX_Z);
    File dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println(datum);
      dataFile.close();
    } else {
      Serial.println("Could not open data file");
    }
  
    Serial.print("Datum: "); Serial.println(datum);
//    Serial.print("RTC time: ");
//    Serial.println(now.timestamp());
//    Serial.print("Temperature: "); Serial.print(temperature); Serial.println("ºC");
//    Serial.print("X: "); Serial.print(PERIOD_MAX_X); Serial.print(' ');
//    Serial.print("Y: "); Serial.print(PERIOD_MAX_Y); Serial.print(' ');
//    Serial.print("Z: "); Serial.print(PERIOD_MAX_Z);
//    Serial.println(" (m/s^2)");
    
    PERIOD_MAX_X = PERIOD_MAX_Y = PERIOD_MAX_Z = 0;
    timer = millis();
  }
}
