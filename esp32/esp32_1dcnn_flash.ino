#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLE2902.h>

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ================= CONFIG =================
#define ALERT_LED 2
#define WINDOW_SIZE 16
#define NUM_FEATURES 9
#define NUM_CLASSES 3
#define SAMPLE_INTERVAL_MS 2000

// BLE UUIDs for the "Nano_DUT" service
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const char* WIFI_SSID = "SOC";
const char* WIFI_PASS = "Isfcr@$OC";

// ================= TRAINED SCALER PARAMETERS =================
// TODO: Replace with the exact StandardScaler mean/std used in training.
// Order MUST match feature order below.
const float scaler_mean[NUM_FEATURES] = {
  0.0f,  // time_ms
  0.0f,  // battery_voltage
  0.0f,  // rssi_dbm
  0.0f,  // ble_conn_attempts
  0.0f,  // prime
  0.0f,  // prime_calc_ms
  0.0f,  // cpu_cycles
  0.0f,  // latency_us
  0.0f   // wifi_attack_hits
};

const float scaler_std[NUM_FEATURES] = {
  1.0f,  // time_ms
  1.0f,  // battery_voltage
  1.0f,  // rssi_dbm
  1.0f,  // ble_conn_attempts
  1.0f,  // prime
  1.0f,  // prime_calc_ms
  1.0f,  // cpu_cycles
  1.0f,  // latency_us
  1.0f   // wifi_attack_hits
};

// ================= TFLITE GLOBALS =================
extern const unsigned char esp32_1dcnn_tflite[] PROGMEM;
extern const unsigned int esp32_1dcnn_tflite_len;

uint8_t* tensor_arena = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
const int kTensorArenaSize = 30 * 1024;

float window_buf[WINDOW_SIZE][NUM_FEATURES] = {0};
int window_idx = 0;
bool window_filled = false;

Adafruit_ADS1115 ads;
BLEScan* pBLEScan;
volatile int ble_rssi = -127;
volatile int conn_attempts = 0;

// ================= BLE CALLBACKS =================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    conn_attempts++;
    Serial.println("!!! BLE CONNECTION DETECTED !!!");
  }

  void onDisconnect(BLEServer* pServer) override {
    pServer->getAdvertising()->start();
  }
};

class MyScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    int rssi = advertisedDevice.getRSSI();
    if (rssi > ble_rssi) ble_rssi = rssi;
  }
};

// ================= HELPERS =================
void softmax(const float* in, float* out, int len) {
  float max_v = in[0];
  for (int i = 1; i < len; i++) {
    if (in[i] > max_v) max_v = in[i];
  }
  float sum = 0.0f;
  for (int i = 0; i < len; i++) {
    out[i] = expf(in[i] - max_v);
    sum += out[i];
  }
  for (int i = 0; i < len; i++) {
    out[i] /= sum;
  }
}

void add_sample_to_window(const float* features) {
  for (int i = 0; i < NUM_FEATURES; i++) {
    window_buf[window_idx][i] = features[i];
  }
  window_idx++;
  if (window_idx >= WINDOW_SIZE) {
    window_idx = 0;
    window_filled = true;
  }
}

void fill_input_tensor() {
  int offset = window_filled ? window_idx : 0;
  int out_idx = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    int src_idx = (offset + i) % WINDOW_SIZE;
    for (int j = 0; j < NUM_FEATURES; j++) {
      float v = window_buf[src_idx][j];
      float norm = (v - scaler_mean[j]) / scaler_std[j];
      input->data.f[out_idx++] = norm;
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, LOW);

  tflite::InitializeTarget();

  tensor_arena = (uint8_t*)heap_caps_malloc(
    kTensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
  );
  if (!tensor_arena) {
    Serial.println("Failed to allocate tensor arena.");
    while (1) delay(1000);
  }

  static tflite::MicroMutableOpResolver<15> resolver;
  resolver.AddConv2D();
  resolver.AddMaxPool2D();
  resolver.AddReshape();
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  resolver.AddRelu();
  resolver.AddQuantize();
  resolver.AddDequantize();
  resolver.AddExpandDims();
  resolver.AddSqueeze();
  resolver.AddMul();
  resolver.AddAdd();
  resolver.AddSub();
  resolver.AddMean();

  const tflite::Model* model = tflite::GetModel(esp32_1dcnn_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch.");
    while (1) delay(1000);
  }

  static tflite::MicroInterpreter static_interpreter(
    model, resolver, tensor_arena, kTensorArenaSize
  );
  interpreter = &static_interpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed.");
    while (1) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  // Hardware Init
  Wire.begin(21, 22);
  ads.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // --- NANO DUT BLE SERVER PART ---
  BLEDevice::init("Nano_DUT");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setValue("Hello Attacker");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks(), false);
  pBLEScan->setActiveScan(false);

  Serial.println("[SYSTEM] Nano_DUT Active & CNN Monitoring Started.");
}

// ================= LOOP =================
void loop() {
  unsigned long currentMillis = millis();

  // Scan for nearby devices (to get RSSI of potential attackers)
  ble_rssi = -127;
  pBLEScan->start(1, false);
  pBLEScan->stop();
  delay(50);

  // Sensors
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = (raw * 0.0000625f) * 5.50f;

  // Features (Time, Volts, RSSI, Conn_Attempts, 5 empty spots)
  float current_features[NUM_FEATURES] = {
    (float)currentMillis,
    voltage,
    (float)ble_rssi,
    (float)conn_attempts,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
  };

  add_sample_to_window(current_features);

  if (window_filled) {
    fill_input_tensor();

    if (interpreter->Invoke() == kTfLiteOk) {
      float probs[NUM_CLASSES];
      softmax(output->data.f, probs, NUM_CLASSES);

      int predicted_class = 0;
      float best = probs[0];
      for (int i = 1; i < NUM_CLASSES; i++) {
        if (probs[i] > best) {
          best = probs[i];
          predicted_class = i;
        }
      }

      bool under_attack = (predicted_class != 0);

      Serial.print("[");
      Serial.print(currentMillis / 1000);
      Serial.print("s] ");

      if (under_attack) {
        digitalWrite(ALERT_LED, HIGH);
        Serial.print("ATTACK");
      } else {
        digitalWrite(ALERT_LED, LOW);
        Serial.print("NORMAL");
      }

      Serial.print(" | V: ");
      Serial.print(voltage, 2);
      Serial.print(" | Hits: ");
      Serial.print(conn_attempts);
      Serial.print(" | Conf: ");
      Serial.println(best, 2);
    }
  } else {
    Serial.print("[");
    Serial.print(currentMillis / 1000);
    Serial.println("s] NORMAL | Buffering...");
    digitalWrite(ALERT_LED, LOW);
  }

  pBLEScan->clearResults();

  if (window_filled) conn_attempts = 0;

  delay(SAMPLE_INTERVAL_MS);
}
