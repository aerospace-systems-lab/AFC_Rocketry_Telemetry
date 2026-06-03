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
// GPS data struct to be created by core 1 and ready by core 0
// =====================================================

struct GPSData {
  int32_t  latitude;
  int32_t  longitude;
  int16_t  altitude;
  uint8_t  satellites;
  uint8_t  fix;
};

GPSData sharedGPS;        // written by Core 1, read by Core 0
mutex_t gpsMutex;         // Pico SDK mutex


// =====================================================
// checksum
// =====================================================
uint8_t calculateChecksum(uint8_t* data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) sum ^= data[i];
  return sum;
}

// =====================================================
// SETUP Core 0
// =====================================================
void setup() {
  mutex_init(&gpsMutex);  // <-- first thing

  Serial.begin(115200);
  delay(1000);

  // ---------------- GPS ----------------
  GPS_SERIAL.setRX(9);
  GPS_SERIAL.setTX(8);
  GPS_SERIAL.begin(115200);

  delay(2000);

  // ---------------- LOGGER ----------------
  LOG_SERIAL.setRX(1);
  LOG_SERIAL.setTX(0);
  LOG_SERIAL.begin(9600);
  delay(1000);
  LOG_SERIAL.print("millis,ax_ms2,ay_ms2,az_ms2,temp_c,baro_alt_m,gps_alt_m,lat,lon,sats,gps_fix\n");

  // ---------------- I2C ----------------
  Wire.begin();
  delay(1000);

  SPI.begin();
  delay(1000);

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

  GPS_SERIAL.print("$PUBX,40,RMC,0,1,0,0*46\r\n");
  GPS_SERIAL.print("$PUBX,40,GGA,0,1,0,0*5B\r\n");
  GPS_SERIAL.print("$PUBX,40,GSA,0,1,0,0*4F\r\n");

  Serial.println("READY");
}


// =====================================================
// SETUP Core 1
// =====================================================
void setup1() {
}


// =====================================================
// LOOP Core 0
// =====================================================
void loop() {

  // =====================================================
  // SENSOR UPDATE
  // =====================================================
  if (sensorTimer.hasPassed(20, true)) {

    GPSData snap;
    mutex_enter_blocking(&gpsMutex);
    snap = sharedGPS;
    mutex_exit(&gpsMutex);
  
    sensors_event_t event;
    lis.getEvent(&event);

    bool bmpOK = bmp.performReading();


    if (bmpOK) {

      float temp = bmp.temperature;
      float baroAlt =
        44330.0f * (1.0f - pow(bmp.pressure / LocalPressure, 0.1903f));

      pkt.xAcc = event.acceleration.x * 100;
      pkt.yAcc = event.acceleration.y * 100;
      pkt.zAcc = event.acceleration.z * 100;
      pkt.temperature = temp * 100;
      pkt.baroAlt = baroAlt * 10;
   

      pkt.latitude   = snap.latitude;
      pkt.longitude  = snap.longitude;
      pkt.gpsAlt     = snap.altitude;
      pkt.satellites = snap.satellites;
      pkt.gpsFix     = snap.fix;

      char line[160];
      snprintf(line, sizeof(line),
        "%lu,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%.6f,%.6f,%u,%u\n",
        millis(),
        event.acceleration.x,
        event.acceleration.y,
        event.acceleration.z,
        temp,
        baroAlt,
        (float)snap.altitude,
        snap.latitude / 1e7f,
        snap.longitude / 1e7f,
        snap.satellites,
        snap.fix
      );

      LOG_SERIAL.print(line);
    }
  }

  // =====================================================
  // RADIO TX (NON-BLOCKING)
  // =====================================================
  if (txTimer.hasPassed(100, true)) {

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

// =====================================================
// LOOP Core 1
// =====================================================

void loop1() {
  while (GPS_SERIAL.available()) {
    if (gps.encode(GPS_SERIAL.read())) {
      // Full sentence decoded — update shared struct
      if (gps.location.isUpdated()) {
        mutex_enter_blocking(&gpsMutex);
        sharedGPS.latitude   = gps.location.lat()  * 1e7;
        sharedGPS.longitude  = gps.location.lng()  * 1e7;
        sharedGPS.fix       = (gps.location.isValid()) ? 3 : 0;
        mutex_exit(&gpsMutex);
      }
      if (gps.altitude.isUpdated()) {
        mutex_enter_blocking(&gpsMutex);
        sharedGPS.altitude   = gps.altitude.meters();
        mutex_exit(&gpsMutex);
      }
      if (gps.satellites.isUpdated()) {
        mutex_enter_blocking(&gpsMutex);
        sharedGPS.satellites = gps.satellites.value();
        mutex_exit(&gpsMutex);
      }
    }
  }
}