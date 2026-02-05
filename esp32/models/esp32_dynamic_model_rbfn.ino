#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <math.h>

// ================= USER CONFIG =================
#define ADS_GAIN GAIN_TWO
#define DIVIDER_RATIO 5.50f
#define SAMPLE_INTERVAL_MS 2000

#define ALERT_LED 2
#define LOCAL_UDP_PORT 4210

#define BLE_SCAN_INTERVAL_MS 2000
#define BLE_SCAN_DURATION_S  1

const char* WIFI_SSID = "SOC";
const char* WIFI_PASS = "Isfcr@$OC";
const char* LAPTOP_IP = "192.168.31.198";
const uint16_t LAPTOP_PORT = 5005;

// ================= RBFN MODEL =================
#define NUM_FEATURES 4   // dv_norm, dv_avg, wifi_rssi, ble_rssi
#define NUM_CENTERS  2

// initial value only used until means stabilize
#define RBF_ATTACK_THRESHOLD 1.00f

// ================= TRAINED PARAMETERS =================
const float scaler_mean_base[NUM_FEATURES] = {
 -1.11e-4, 2.74e-6, -50.1f, -65.0f
};

const float scaler_std_base[NUM_FEATURES] = {
 0.0082f, 0.1028f, 8.13f, 10.0f
};

float centers[NUM_CENTERS][NUM_FEATURES] = {
  {  0.91f,  0.94f,  0.74f,  0.50f },
  { -0.89f, -0.93f, -0.73f, -0.50f }
};

const float rbf_sigma = 3.48499518;
float output_weights[NUM_CENTERS] = { 8.88f, -9.78f };
float output_bias = 0.83276382;

// ================= LEARNING RATES =================
#define RBF_LR     0.05f
#define MEAN_LR    0.05f   // class-mean adaptation rate

// ================= THRESHOLD REGULATION =================
#define THRESH_LR_UP     0.08f    // threshold rises fast (attack correction)
#define THRESH_LR_DOWN   0.002f   // threshold falls slowly (normal drift)
#define THRESH_MAX_STEP  0.02f    // absolute max change per loop

// ================= GLOBAL OBJECTS =================
Adafruit_ADS1115 ads;
WiFiUDP udp;
BLEScan* pBLEScan;

// ================= BLE STATE =================
volatile int ble_rssi = -127;
unsigned long last_ble_scan = 0;
volatile int conn_attempts = 0;

// ================= SLOPE SMOOTHING =================
#define SLOPE_WIN 5
float dv_buf[SLOPE_WIN] = {0};
int dv_idx = 0;
float prev_voltage = 0.0f;

// ================= ADAPTIVE SEPARATION =================
float mean_normal = 0.0f;
float mean_attack = 0.0f;
float adaptive_threshold = RBF_ATTACK_THRESHOLD;
bool means_initialized = false;

// ================= BLE SCAN CALLBACK =================
class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    int rssi = advertisedDevice.getRSSI();
    if (rssi > ble_rssi) {
      ble_rssi = rssi;   // strongest RSSI in window
    }
  }
};

// ================= BLE SERVER CALLBACK =================
class SecurityCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    conn_attempts++;
  }

  void onDisconnect(BLEServer* pServer) {
    pServer->getAdvertising()->start();
  }
};

// ================= RBF FUNCTIONS =================
float rbf(float* x, const float* c) {
  float d2 = 0.0f;
  for (int i = 0; i < NUM_FEATURES; i++) {
    float d = x[i] - c[i];
    d2 += d * d;
  }
  return exp(-d2 / (2.0f * rbf_sigma * rbf_sigma));
}

float rbfn_score(float* f) {
  float x[NUM_FEATURES];
  for (int i = 0; i < NUM_FEATURES; i++)
    x[i] = (f[i] - scaler_mean_base[i]) / scaler_std_base[i];

  float score = output_bias;
  for (int i = 0; i < NUM_CENTERS; i++)
    score += output_weights[i] * rbf(x, centers[i]);

  return score;
}

void rbfn_update(float* f, float error) {
  float x[NUM_FEATURES];
  for (int i = 0; i < NUM_FEATURES; i++)
    x[i] = (f[i] - scaler_mean_base[i]) / scaler_std_base[i];

  for (int i = 0; i < NUM_CENTERS; i++)
    output_weights[i] += RBF_LR * error * rbf(x, centers[i]);

  output_bias += RBF_LR * error;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  pinMode(ALERT_LED, OUTPUT);

  Wire.begin(21, 22);
  ads.begin();
  ads.setGain(ADS_GAIN);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  udp.begin(LOCAL_UDP_PORT);

  BLEDevice::init("Nano_DUT");

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SecurityCallbacks());
  BLEDevice::getAdvertising()->start();

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(50);
}

// ================= LOOP =================
void loop() {

  // ---- Periodic BLE RSSI Scan ----
  if (millis() - last_ble_scan >= BLE_SCAN_INTERVAL_MS) {
    last_ble_scan = millis();
    ble_rssi = -127;
    pBLEScan->start(BLE_SCAN_DURATION_S, false);
  }

  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = (raw * 0.0000625f) * DIVIDER_RATIO;

  float dv = voltage - prev_voltage;
  prev_voltage = voltage;

  if (dv < 0.0f)
    dv_buf[dv_idx++ % SLOPE_WIN] = dv;
  else
    memset(dv_buf, 0, sizeof(dv_buf));

  float dv_avg = 0.0f;
  for (int i = 0; i < SLOPE_WIN; i++)
    dv_avg += dv_buf[i];
  dv_avg /= SLOPE_WIN;

  float dv_norm = (voltage > 0.1f) ? dv_avg / voltage : 0.0f;
  int wifi_rssi = WiFi.RSSI();

  float features[NUM_FEATURES] = {
    dv_norm, dv_avg, (float)wifi_rssi, (float)ble_rssi
  };

  float score = rbfn_score(features);

  // ---- Supervised RBF update (unchanged) ----
  rbfn_update(features, (conn_attempts > 0 ? 1.0f : 0.0f) - score);

  // ---- Initialize means safely ----
  if (!means_initialized) {
    mean_normal = score;
    mean_attack = score;
    means_initialized = true;
  }

  // ---- Weak supervision for class means ----
  if (conn_attempts > 0) {
    mean_attack += MEAN_LR * (score - mean_attack);
  } else {
    mean_normal += MEAN_LR * (score - mean_normal);
  }

  // ---- Adaptive decision boundary ----
  // ---- Regulated adaptive decision boundary ----
  float desired_threshold = 0.5f * (mean_attack + mean_normal);

  // error between where we are and where we want to be
  float err = desired_threshold - adaptive_threshold;

  // asymmetric learning rate
  float lr = (err > 0.0f) ? THRESH_LR_UP : THRESH_LR_DOWN;

  // proposed update
  float delta = lr * err;

  // rate limiting (hard cap)
  if (delta > THRESH_MAX_STEP)  delta = THRESH_MAX_STEP;
  if (delta < -THRESH_MAX_STEP) delta = -THRESH_MAX_STEP;

  // apply update
  adaptive_threshold += delta;


  bool under_attack = score > adaptive_threshold;

  digitalWrite(ALERT_LED, under_attack ? !digitalRead(ALERT_LED) : LOW);

  char packet[256];
  snprintf(packet, sizeof(packet),
    "%s,%lu,%.3f,%.6f,%.6f,%d,%d,%.4f,%.4f,%d",
    under_attack ? "ATTACK" : "NORMAL",
    millis(), voltage, dv_norm, dv_avg,
    wifi_rssi, ble_rssi,
    score, adaptive_threshold, conn_attempts
  );

  udp.beginPacket(LAPTOP_IP, LAPTOP_PORT);
  udp.write((uint8_t*)packet, strlen(packet));
  udp.endPacket();

  Serial.println(packet);
  conn_attempts = 0;

  delay(SAMPLE_INTERVAL_MS);
}