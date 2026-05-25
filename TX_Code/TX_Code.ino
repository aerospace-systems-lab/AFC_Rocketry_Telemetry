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
// GPS
// =====================================================
#define GPS_SERIAL Serial2
TinyGPSPlus gps;

// =====================================================
// Logger
// =====================================================
#define LOG_SERIAL Serial1
OpenLogManager myLogger(LOG_SERIAL);

// =====================================================
// RADIO
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
// RADIO STATE
// =====================================================
bool txBusy = false;

// =====================================================
// TELEMETRY
// =====================================================
#pragma pack(push, 1)
struct TelemetryPacket {
  uint16_t magic;
  uint8_t version;
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

// =====================================================
// GPS drain (FIXED: time-based, not count-based)
// =====================================================
inline void pollGPS() {
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }
}


// =====================================================
// checksum
// =====================================================
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

  // ---------------- GPS ----------------
  GPS_SERIAL.setRX(9);
  GPS_SERIAL.setTX(8);
  GPS_SERIAL.begin(115200);

  unsigned long t0 = millis();
while (millis() - t0 < 2000) {
    pollGPS();   // let GPS boot and send initial sentences
}
  delay(2000);

  // Minimal + stable config (IMPORTANT)
  GPS_SERIAL.print("$PUBX,40,RMC,0,1,0,0*46\r\n");
  GPS_SERIAL.print("$PUBX,40,GGA,0,1,0,0*5B\r\n");
  GPS_SERIAL.print("$PUBX,40,GSA,0,1,0,0*4F\r\n");

  // ---------------- LOGGER ----------------
  LOG_SERIAL.setRX(1);
  LOG_SERIAL.setTX(0);
  LOG_SERIAL.begin(9600);

  // ---------------- I2C ----------------
  Wire.begin();
  SPI.begin();

  // ---------------- LIS ----------------
  if (!lis.begin(0x18)) {
    Serial.println("LIS FAIL");
    while (1);
  }
  lis.setRange(LIS3DH_RANGE_16_G);

  // ---------------- BMP ----------------
  if (!bmp.begin_I2C()) {
    Serial.println("BMP FAIL");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  while (true) {
    if (bmp.performReading()) {
      LocalPressure = bmp.pressure;
      if (LocalPressure > 95000) break;
    }
    delay(50);
  }

  // ---------------- RADIO ----------------
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("RADIO FAIL");
    while (1);
  }

  radio.setFrequency(2400.0);
  radio.setBandwidth(406);
  radio.setSpreadingFactor(7);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setOutputPower(10);
  radio.setCRC(true);

  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = 0xABCD;
  pkt.version = 1;

  Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // =====================================================
  // ALWAYS SERVICE GPS FIRST
  // =====================================================
  pollGPS();

  // =====================================================
  // SENSOR UPDATE
  // =====================================================
  if (sensorTimer.hasPassed(20, true)) {

    pollGPS();

    sensors_event_t event;
    lis.getEvent(&event);

    bool bmpOK = bmp.performReading();

    pollGPS();

    if (bmpOK) {

      float temp = bmp.temperature;
      float baroAlt =
        44330.0f * (1.0f - pow(bmp.pressure / LocalPressure, 0.1903f));

      pkt.xAcc = event.acceleration.x * 100;
      pkt.yAcc = event.acceleration.y * 100;
      pkt.zAcc = event.acceleration.z * 100;
      pkt.temperature = temp * 100;
      pkt.baroAlt = baroAlt * 10;

      // =====================================================
      // GPS FIX HANDLING (FIXED LOGIC)
      // =====================================================

      static int32_t lastLat = 0;
      static int32_t lastLon = 0;
      static int16_t lastAlt = 0;
      static uint8_t lastSats = 0;

      if (gps.location.isUpdated()) {
        lastLat = gps.location.lat() * 1e7;
        lastLon = gps.location.lng() * 1e7;
      }

      if (gps.altitude.isUpdated()) {
        lastAlt = gps.altitude.meters();
      }

      if (gps.satellites.isUpdated()) {
        lastSats = gps.satellites.value();
      }

      pkt.latitude = lastLat;
      pkt.longitude = lastLon;
      pkt.gpsAlt = lastAlt;
      pkt.satellites = lastSats;

      // FIX STATE (simple but stable)
      pkt.gpsFix = (lastLat != 0 && lastLon != 0) ? 3 : 0;

      char line[160];
      snprintf(line, sizeof(line),
        "%lu,%.3f,%.3f,%.3f,%.2f,%.1f,%.6f,%.6f,%u,%u\n",
        millis(),
        event.acceleration.x / 9.81f,
        event.acceleration.y / 9.81f,
        event.acceleration.z / 9.81f,
        temp,
        baroAlt,
        pkt.latitude / 1e7f,
        pkt.longitude / 1e7f,
        pkt.satellites,
        pkt.gpsFix
      );

      LOG_SERIAL.print(line);
    }
  }

  // =====================================================
  // RADIO TX (NON-BLOCKING)
  // =====================================================
  if (txTimer.hasPassed(100, true)) {

    pollGPS();

    if (txBusy) {
      int state = radio.getIrqFlags();

      if (state & RADIOLIB_SX128X_IRQ_TX_DONE) {
        radio.clearIrqFlags(RADIOLIB_SX128X_IRQ_TX_DONE);
        txBusy = false;
      }
    }

    static uint16_t counter = 0;
    pkt.counter = counter++;
    pkt.checksum = calculateChecksum((uint8_t*)&pkt, sizeof(pkt) - 1);

    int txState = radio.startTransmit((uint8_t*)&pkt, sizeof(pkt));

    if (txState == RADIOLIB_ERR_NONE) {
      txBusy = true;
    }
  }
}