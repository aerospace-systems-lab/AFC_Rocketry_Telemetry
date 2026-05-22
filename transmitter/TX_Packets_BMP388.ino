#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <OpenLogManager.h>

// =====================================================
// GPS UART
// =====================================================
#define GPS_SERIAL Serial2
#define LOG_SERIAL Serial1
OpenLogManager myLogger(LOG_SERIAL);
// =====================================================
// UBX parser state machine
// =====================================================
uint8_t state = 0;
uint8_t msgClass = 0;
uint8_t msgId = 0;
uint16_t length = 0;
uint16_t UBXindex = 0;
uint8_t payload[128];

uint8_t ckA = 0, ckB = 0;          // <-- ADD THESE

volatile int32_t lat_latched = 0;
volatile int32_t lon_latched = 0;
volatile int32_t alt_latched = 0;
volatile uint8_t fix_latched = 0;
volatile uint8_t sv_latched = 0;

// =====================================================
// GPS data structure
// =====================================================
struct GPSData {
  int32_t lat;
  int32_t lon;
  int32_t hMSL;

  uint8_t fixType;
  uint8_t numSV;

  uint8_t hour, minute, second;
  uint16_t year;
  uint8_t month, day;

  uint32_t gSpeed;
  int32_t headMot;
};

GPSData gps;

// =====================================================
// NAV-PVT parser
// =====================================================
void parseNavPVT(uint8_t *p) {

  gps.year   = p[4] | (p[5] << 8);
  gps.month  = p[6];
  gps.day    = p[7];

  gps.hour   = p[8];
  gps.minute = p[9];
  gps.second = p[10];

  gps.fixType = p[20];
  gps.numSV   = p[23];

  gps.lon = (int32_t)(
    (uint32_t)p[24] |
    ((uint32_t)p[25] << 8) |
    ((uint32_t)p[26] << 16) |
    ((uint32_t)p[27] << 24)
  );

  gps.lat = (int32_t)(
    (uint32_t)p[28] |
    ((uint32_t)p[29] << 8) |
    ((uint32_t)p[30] << 16) |
    ((uint32_t)p[31] << 24)
  );

  gps.hMSL = (int32_t)(
    (uint32_t)p[36] |
    ((uint32_t)p[37] << 8) |
    ((uint32_t)p[38] << 16) |
    ((uint32_t)p[39] << 24)
  );

  gps.gSpeed = (uint32_t)(
    (uint32_t)p[60] |
    ((uint32_t)p[61] << 8) |
    ((uint32_t)p[62] << 16) |
    ((uint32_t)p[63] << 24)
  );

  gps.headMot = (int32_t)(
    (uint32_t)p[64] |
    ((uint32_t)p[65] << 8) |
    ((uint32_t)p[66] << 16) |
    ((uint32_t)p[67] << 24)
  );
}

// =====================================================
// UBX reader
// =====================================================
void readUBX() {

  while (GPS_SERIAL.available()) {

    uint8_t b = GPS_SERIAL.read();

    switch (state) {

      case 0: if (b == 0xB5) state = 1; break;
      case 1: state = (b == 0x62) ? 2 : 0; break;

      case 2: msgClass = b; state = 3; break;
      case 3: msgId = b; state = 4; break;

      case 4:
        length = b;
        state = 5;
        break;

      case 5:
        length |= (uint16_t)b << 8;
        UBXindex = 0;
        state = (length <= sizeof(payload)) ? 6 : 0;
        break;

      case 6:
        payload[UBXindex++] = b;
        if (UBXindex >= length) state = 7;
        break;

      case 7:  // first checksum byte
  ckA = b;
  state = 8;
  break;

case 8:  // second checksum byte — now validate and parse
  ckB = b;
  {
    // Compute expected checksum over class, id, length bytes, and payload
    uint8_t cA = 0, cB = 0;
    cA += msgClass; cB += cA;
    cA += msgId;    cB += cA;
    cA += (length & 0xFF);       cB += cA;
    cA += (length >> 8) & 0xFF;  cB += cA;
    for (uint16_t i = 0; i < length; i++) {
      cA += payload[i];
      cB += cA;
    }

    if (cA == ckA && cB == ckB) {
      if (msgClass == 0x01 && msgId == 0x07) {
        parseNavPVT(payload);
        if (gps.fixType >= 3) {
          lat_latched = gps.lat;
          lon_latched = gps.lon;
          alt_latched = gps.hMSL;
          fix_latched = gps.fixType;
          sv_latched  = gps.numSV;
        }
      }
    }
  }
  state = 0;
  break;
    }
  }
}

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
// I2C SCANNER (DEBUG TOOL)
// =====================================================
void scanI2C() {
  Serial.println("🔍 I2C scanning...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }

  Serial.println("Done.");
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
  radio.setBandwidth(812.5);
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

  readUBX();

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

    pkt.satellites = sv_latched;
    pkt.gpsFix = fix_latched;

    if (fix_latched >= 3 && sv_latched >= 6) {
      pkt.latitude = lat_latched;
      pkt.longitude = lon_latched;
      pkt.gpsAlt = alt_latched / 1000;
    } else {
      pkt.latitude = 0;
      pkt.longitude = 0;
      pkt.gpsAlt = 0;
    }
    char line[160];

    snprintf(line, sizeof(line),
    "%lu,%.3f,%.3f,%.3f,%.2f,%.1f,%ld,%ld,%u,%u\n",
    millis(),
    event.acceleration.x / 9.81f,   // G
    event.acceleration.y / 9.81f,
    event.acceleration.z / 9.81f,
    temp,                             // °C
    baroAlt,                          // m
    pkt.latitude,                     // raw int32 (1e-7 deg)
    pkt.longitude,
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