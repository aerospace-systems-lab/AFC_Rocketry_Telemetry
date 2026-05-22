#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <OpenLogManager.h>
#include <TinyGPSPlus.h>

// =====================================================
// GPS UART
// =====================================================
#define GPS_SERIAL Serial2
TinyGPSPlus gps;

// =====================================================
// Logger UART
// =====================================================
#define LOG_SERIAL Serial1
OpenLogManager myLogger(LOG_SERIAL);

// =====================================================
// SX1280 RADIO
// =====================================================
SX1280 radio = new Module(17, 13, 15, 14);

// =====================================================
// SENSORS
// =====================================================
Adafruit_LIS3DH lis;
Adafruit_BMP3XX bmp;
float LocalPressure = 0;

// =====================================================
// TIMERS
// =====================================================
#include <Chrono.h>
Chrono sensorTimer;
Chrono txTimer;

// =====================================================
// TELEMETRY PACKET
// =====================================================
#pragma pack(push, 1)
struct TelemetryPacket {

  uint16_t magic;
  uint8_t  version;

  int16_t xAcc;
  int16_t yAcc;
  int16_t zAcc;

  int16_t temperature;

  int16_t baroAlt;
  int16_t gpsAlt;

  int32_t latitude;
  int32_t longitude;

  uint8_t satellites;
  uint8_t gpsFix;

  uint16_t counter;
  uint8_t checksum;
};
#pragma pack(pop)

TelemetryPacket pkt;

uint8_t calculateChecksum(uint8_t* data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) sum ^= data[i];
  return sum;
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);
  delay(1000);

  // GPS UART
  GPS_SERIAL.setRX(9);
  GPS_SERIAL.setTX(8);
  GPS_SERIAL.begin(115200);

  // Sparkfun OpenLog UART
  LOG_SERIAL.setRX(1);
  LOG_SERIAL.setTX(0);
  LOG_SERIAL.begin(9600);

  // I2C + SPI
  Wire.begin();
  SPI.begin();

  delay(500);

  // =====================================================
  // LIS3DH INIT
  // =====================================================
  if (!lis.begin(0x18)) {
    Serial.println("LIS3DH not found at 0x18");
    while (1) delay(10);
  }
  Serial.println("LIS3DH detected");
  lis.setRange(LIS3DH_RANGE_16_G);

  // =====================================================
  // BMP388 INIT
  // =====================================================
  if (!bmp.begin_I2C()) {
    Serial.println("BMP388 Not Found");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(50);

 do {
    while (!bmp.performReading()) { delay(100); }
    LocalPressure = bmp.readPressure();
    Serial.print("Reading: ");
    Serial.println(LocalPressure / 100.0f);
} while (LocalPressure < 95000.0f); // reject anything below ~950 hPa as bogus

  Serial.println("Sensors initialized");

  // =====================================================
  // RADIO INIT
  // =====================================================
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("SX1280 FAIL");
    while (1);
  }

  radio.setFrequency(2400.0);
  radio.setBandwidth(406);
  radio.setSpreadingFactor(7);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setOutputPower(10);
  radio.setCRC(true);

  Serial.println("TX READY");

  // INIT Logger

  LOG_SERIAL.println("ms,ax_g,ay_g,az_g,temp_c,baro_m,gpsLat,gpsLon,sats,fix");
  delay(5000);

  // INIT PACKET
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = 0xABCD;
  pkt.version = 1;
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // Feed all available GPS bytes to TinyGPSPlus
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // 50Hz SENSOR UPDATE
  if (sensorTimer.hasPassed(20, true)) {

    sensors_event_t event;
    lis.getEvent(&event);

    bmp.performReading();

    float temp = bmp.temperature;
    float baroAlt = 44330.0f * (1.0f - pow(bmp.pressure / LocalPressure, 0.1903f));

    pkt.xAcc = event.acceleration.x * 100;
    pkt.yAcc = event.acceleration.y * 100;
    pkt.zAcc = event.acceleration.z * 100;

    pkt.temperature = temp * 100;
    pkt.baroAlt = baroAlt * 10;

    // GPS: require a valid fix with at least 6 satellites
    bool goodFix = gps.location.isValid()
                && gps.satellites.isValid()
                && gps.satellites.value() >= 6;

    if (goodFix) {
      // Convert double degrees → int32 in units of 1e-7 degrees, matching your old UBX format
      pkt.latitude   = (int32_t)(gps.location.lat() * 1e7);
      pkt.longitude  = (int32_t)(gps.location.lng() * 1e7);
      pkt.gpsAlt     = gps.altitude.isValid() ? (int16_t)(gps.altitude.meters()) : 0;
      pkt.satellites = (uint8_t)gps.satellites.value();
      pkt.gpsFix     = 3;   // TinyGPSPlus doesn't expose UBX fix type; 3 = 3D fix equivalent
    } else {
      pkt.latitude   = 0;
      pkt.longitude  = 0;
      pkt.gpsAlt     = 0;
      pkt.satellites = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
      pkt.gpsFix     = 0;
    }

    char line[160];

    snprintf(line, sizeof(line),
    "%lu,%.3f,%.3f,%.3f,%.2f,%.1f,%.6f,%.6f,%u,%u\n",
    millis(),
    event.acceleration.x / 9.81f,   // G
    event.acceleration.y / 9.81f,
    event.acceleration.z / 9.81f,
    temp,                             // °C
    baroAlt,                          // m
    pkt.latitude / 1e7f,                     // float, in deg 
    pkt.longitude / 1e7f,
    pkt.satellites,
    pkt.gpsFix
);

LOG_SERIAL.print(line);
  }
  // 10Hz TX
  if (txTimer.hasPassed(100, true)) {

    static uint16_t counter = 0;

    pkt.counter = counter++;
    pkt.checksum = calculateChecksum((uint8_t*)&pkt, sizeof(pkt) - 1);

    int txState = radio.transmit((uint8_t*)&pkt, sizeof(pkt));

Serial.print("TX state = ");
Serial.println(txState);

if (txState != RADIOLIB_ERR_NONE) {
  Serial.println("TX FAIL");
} else {
  Serial.println("TX OK");
}
  }
}