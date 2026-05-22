/*
  SX1280 Binary Telemetry Receiver (FIXED + SAFE)
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// =========================
// SX1280 CONFIG
// =========================

SX1280 radio = new Module(17, 13, 15, 14);

// =========================
// INTERRUPT FLAG
// =========================

volatile bool receivedFlag = false;

void setFlag(void) {
  receivedFlag = true;
}

// =========================
// TELEMETRY PACKET (MUST MATCH TX EXACTLY)
// =========================

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

// =========================
// STATE
// =========================

uint16_t lastCounter = 0;

// =========================
// CHECKSUM
// =========================

uint8_t calculateChecksum(uint8_t* data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    sum ^= data[i];
  }
  return sum;
}

// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);
  delay(2000);

  SPI.begin();
  delay(500);

  Serial.print("[SX1280] Initializing... ");

  int state = radio.begin();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("FAILED, code ");
    Serial.println(state);
    while (true) delay(10);
  }


  radio.setFrequency(2400.0);
  radio.setBandwidth(812.5);
  radio.setSpreadingFactor(7);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setCRC(true);

  Serial.println("OK");

  radio.setPacketReceivedAction(setFlag);

  Serial.print("[SX1280] Starting RX... ");

  state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("FAILED, code ");
    Serial.println(state);
    while (true) delay(10);
  }

  Serial.println("OK");
}

// =========================
// LOOP
// =========================

void loop() {

  if (!receivedFlag) return;
  receivedFlag = false;

  int state = radio.readData((uint8_t*)&pkt, sizeof(pkt));

  // =========================
  // VALID PACKET ONLY
  // =========================

  if (state == RADIOLIB_ERR_NONE) {

    // -------------------------
    // MAGIC + VERSION CHECK
    // -------------------------

    if (pkt.magic != 0xABCD || pkt.version != 1) {
      return; // drop corrupted packet
    }

    // -------------------------
    // CHECKSUM VERIFY
    // -------------------------

    uint8_t calc = calculateChecksum((uint8_t*)&pkt, sizeof(pkt) - 1);

    if (calc != pkt.checksum) {
      Serial.println("#ERR,bad_checksum");
      return;
    }

    // -------------------------
    // PACKET LOSS DETECTION
    // -------------------------

    if (lastCounter != 0 && pkt.counter != lastCounter + 1) {
      Serial.println("#WARN,packet_loss");
    }

    lastCounter = pkt.counter;

    // -------------------------
    // UNIT CONVERSIONS
    // -------------------------

    float xAcc = pkt.xAcc / 100.0f / 9.81f;
    float yAcc = pkt.yAcc / 100.0f / 9.81f;
    float zAcc = pkt.zAcc / 100.0f / 9.81f;

    float temperature = pkt.temperature / 100.0;

    float baroAlt = pkt.baroAlt / 10.0;

    // GPS altitude: cm → meters
    float gpsAlt = (float)pkt.gpsAlt;

    bool gpsValid = (pkt.gpsFix >= 3 && pkt.satellites >= 6);

    double lat = 0;
    double lon = 0;

    if (gpsValid) {
      lat = pkt.latitude * 1e-7;
      lon = pkt.longitude * 1e-7;
    }

    // -------------------------
    // CSV OUTPUT
    // -------------------------

    Serial.print(pkt.counter); Serial.print(",");

    Serial.print(xAcc); Serial.print(",");
    Serial.print(yAcc); Serial.print(",");
    Serial.print(zAcc); Serial.print(",");

    Serial.print(temperature); Serial.print(",");
    Serial.print(baroAlt); Serial.print(",");
    Serial.print(gpsAlt); Serial.print(",");

    Serial.print(pkt.gpsFix); Serial.print(",");

    Serial.print(lat, 7); Serial.print(",");
    Serial.print(lon, 7); Serial.print(",");

    Serial.print(pkt.satellites); Serial.print(",");

    Serial.print(radio.getRSSI()); Serial.print(",");
    Serial.print(radio.getSNR()); Serial.print(",");
    Serial.print(radio.getFrequencyError());

    Serial.println();
  }

  // =========================
  // ERROR HANDLING
  // =========================

  else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("#ERR,crc");
  }

  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println("#ERR,packet_too_long");
  }

  else {
    Serial.print("#ERR,rx,");
    Serial.println(state);
  }

  // =========================
  // RESTART RX
  // =========================

  radio.startReceive();
}