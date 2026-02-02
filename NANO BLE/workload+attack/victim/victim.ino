#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoBLE.h>

// ================= ADS1115 =================
Adafruit_ADS1115 ads;
bool adsOk = false;

const float R1 = 30000.0;
const float R2 = 7500.0;
const float BATTERY_SCALE = (R1 + R2) / R2;
const float ADS_LSB = 0.000125; // GAIN_ONE

// ================= BLE UUIDS =================
BLEService telemService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEUnsignedShortCharacteristic battmVChar(
  "19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

BLEUnsignedIntCharacteristic advCountChar(
  "19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

BLEUnsignedShortCharacteristic connEvtChar(
  "19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// ================= TIMING =================
const unsigned long UPDATE_MS = 200;
unsigned long lastUpdate = 0;

// ================= METRICS =================
uint32_t advCount = 0;
uint16_t connEvents = 0;

float readBatteryVoltage() {
  int16_t adc = ads.readADC_SingleEnded(0);
  float v_adc = adc * ADS_LSB;
  return v_adc * BATTERY_SCALE;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();
  adsOk = ads.begin(0x48);  // change if needed
  ads.setGain(GAIN_ONE);

  if (!BLE.begin()) {
    while (1);
  }

  BLE.setDeviceName("Nano_DUT");
  BLE.setLocalName("Nano_DUT");

  BLE.setAdvertisedService(telemService);
  telemService.addCharacteristic(battmVChar);
  telemService.addCharacteristic(advCountChar);
  telemService.addCharacteristic(connEvtChar);
  BLE.addService(telemService);

  battmVChar.writeValue((uint16_t)0);
  advCountChar.writeValue((uint32_t)0);
  connEvtChar.writeValue((uint16_t)0);

  BLE.advertise();
}

void loop() {
  BLE.poll();

  // conn request approx = connection events
  BLEDevice central = BLE.central();
  if (central) {
    connEvents++;
    while (central.connected()) {
      BLE.poll();
      delay(2);
    }
  }

  if (millis() - lastUpdate >= UPDATE_MS) {
    lastUpdate = millis();
    advCount++;

    float battV = adsOk ? readBatteryVoltage() : 0.0;
    uint16_t battmV = (uint16_t)(battV * 1000.0);

    battmVChar.writeValue(battmV);
    advCountChar.writeValue(advCount);
    connEvtChar.writeValue(connEvents);
  }
}
