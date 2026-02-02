#include <ArduinoBLE.h>

#define TARGET_NAME "Nano_DUT"
#define RSSI_BUF 20
#define POLL_MS 1000

int rssiBuf[RSSI_BUF];
int rssiIdx = 0;
int rssiCount = 0;

BLEDevice dut;

// UUIDs
const char* SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* BATT_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
const char* ADV_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214";
const char* CONN_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214";

unsigned long lastPoll = 0;

float meanRSSI() {
  if (rssiCount == 0) return 0;
  long sum = 0;
  for (int i = 0; i < rssiCount; i++) sum += rssiBuf[i];
  return (float)sum / (float)rssiCount;
}

float varRSSI() {
  if (rssiCount < 2) return 0;
  float m = meanRSSI();
  float sumSq = 0;
  for (int i = 0; i < rssiCount; i++) {
    float d = (float)rssiBuf[i] - m;
    sumSq += d * d;
  }
  return sumSq / (float)(rssiCount - 1);
}

bool readDutTelemetry(float& battV, uint32_t& advCount, uint16_t& connEvents) {
  battV = -1.0;
  advCount = 0;
  connEvents = 0;

  if (!dut) return false;
  if (!dut.connect()) return false;

  if (!dut.discoverAttributes()) {
    dut.disconnect();
    return false;
  }

  BLEService s = dut.service(SERVICE_UUID);
  if (!s) {
    dut.disconnect();
    return false;
  }

  BLECharacteristic battC = s.characteristic(BATT_UUID);
  BLECharacteristic advC = s.characteristic(ADV_UUID);
  BLECharacteristic connC = s.characteristic(CONN_UUID);

  if (!battC || !advC || !connC) {
    dut.disconnect();
    return false;
  }

  uint16_t battmV = 0;
  uint32_t adv = 0;
  uint16_t conn = 0;

  bool ok = true;
  if (!battC.readValue(battmV)) ok = false;
  if (!advC.readValue(adv)) ok = false;
  if (!connC.readValue(conn)) ok = false;

  dut.disconnect();

  if (!ok) return false;

  battV = battmV / 1000.0;
  advCount = adv;
  connEvents = conn;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!BLE.begin()) {
    while (1)
      ;
  }

  BLE.setDeviceName("LoggerNode");
  BLE.setLocalName("LoggerNode");

  BLE.scan();

  Serial.println("time_ms,batt_V,advCount,connEvents,rssi_mean,rssi_var");
}

void loop() {
  BLE.poll();

  // collect RSSI samples from advertisements
  BLEDevice dev = BLE.available();
  if (dev && dev.localName() == TARGET_NAME) {
    int rssi = dev.rssi();

    rssiBuf[rssiIdx] = rssi;
    rssiIdx = (rssiIdx + 1) % RSSI_BUF;
    if (rssiCount < RSSI_BUF) rssiCount++;

    dut = dev;  // keep latest handle
  }

  // log once per second
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();

    BLE.stopScan();
    float battV;
    uint32_t advCount;
    uint16_t connEvents;
    bool ok = readDutTelemetry(battV, advCount, connEvents);
    BLE.scan();

    Serial.print(millis());
    Serial.print(",");

    if (ok) {
      Serial.print(battV, 3);
      Serial.print(",");
      Serial.print(advCount);
      Serial.print(",");
      Serial.print(connEvents);
    } else {
      Serial.print("-1.000,0,0");
    }

    Serial.print(",");
    Serial.print(meanRSSI(), 2);
    Serial.print(",");
    Serial.println(varRSSI(), 2);
  }
}
