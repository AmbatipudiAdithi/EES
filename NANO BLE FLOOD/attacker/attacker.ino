#include <ArduinoBLE.h>

#define NODE_ID       "A"
#define TARGET_NAME   "Nano_DUT"
#define HOLD_MS       400
#define IDLE_MS       100

bool scanning = false;

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("ERROR: BLE.begin failed");
    while (1);
  }

  char n[32];
  snprintf(n, sizeof(n), "LoadNode_%s", NODE_ID);
  BLE.setLocalName(n);
  BLE.setDeviceName(n);

  Serial.print("Load tester started: ");
  Serial.println(n);

  startScan();
}

void startScan() {
  if (scanning) return;
  if (BLE.scan()) {
    scanning = true;
  } else {
    Serial.println("Scan start failed");
  }
}

void loop() {
  BLE.poll();

  if (!scanning) {
    startScan();
    return;
  }

  BLEDevice dev = BLE.available();
  if (!dev) return;

  // Proper string comparison for ArduinoBLE
  String localName = dev.localName();
  if (localName != TARGET_NAME) return;

  Serial.print("Target seen RSSI=");
  Serial.print(dev.rssi());
  Serial.print(" addr=");
  Serial.println(dev.address());

  BLE.stopScan();  // IMPORTANT
  scanning = false;

  digitalWrite(LED_BUILTIN, HIGH);

  if (dev.connect()) {
    Serial.println("Connected");

    unsigned long t0 = millis();
    while (dev.connected() && millis() - t0 < HOLD_MS) {
      BLE.poll();
      delay(1);
    }

    if (dev.connected()) {
      dev.disconnect();
    }
    Serial.println("Disconnected");
  } else {
    Serial.println("Connect failed");
  }

  digitalWrite(LED_BUILTIN, LOW);

  delay(IDLE_MS + random(0, 25));

  startScan();  // restart scan
}
