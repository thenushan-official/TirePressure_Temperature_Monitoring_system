#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// GPIO Assignments
#define NTC_PIN 34         // ADC pin for NTC thermistor (GPIO34)
#define MPX5700_PIN 35     // ADC pin for MPX5700 sensor (GPIO35)
#define LED_PIN 2          // Onboard LED (GPIO2)

// Constants
#define VS 3.3             // Supply voltage for sensors
#define R_FIXED 10000      // Fixed resistor value (10kΩ)
#define ADC_RESOLUTION 4095.0 // 12-bit ADC resolution for ESP32

// Steinhart-Hart coefficients for 10k NTC thermistor
#define A 0.001129148
#define B 0.000234125
#define C 0.0000000876741


// Struct to hold sensor data
typedef struct {
  float temperature;
  float pressure;
} SensorData;

// Initialize sensor data
SensorData dataToSend;

// Function to calculate pressure from MPX5700 sensor output
float calculatePressure(float analogValue) {
  // Step 1: Convert ADC value to sensor output voltage (Vout)
  float vout = ((analogValue * VS) /  ADC_RESOLUTION) + 0.2;
  float pressure = (160*vout-40)*0.145038;
  return pressure; // Pressure in kPa
}

// Function to calculate temperature from NTC thermistor
float calculateTemperature(float analogValue) {
  float voltage = analogValue * (VS / ADC_RESOLUTION);  // Convert ADC value to voltage
  if (voltage == 0) return -273.15;  // Prevent division by zero

  float resistance = (VS / voltage - 1) * R_FIXED;  // Calculate resistance
  float logR = log(resistance);
  float temperatureKelvin = 1.0 / (A + B * logR + C * logR * logR * logR);  // Steinhart-Hart
  return temperatureKelvin - 273.15;  // Convert Kelvin to Celsius
}

// Callback for ESP-NOW send status
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Data send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

  // Blink onboard LED to indicate data was sent
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Configure GPIO pins
  pinMode(NTC_PIN, INPUT);
  pinMode(MPX5700_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure LED is off initially

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Add ESP-NOW peer
  uint8_t peerAddress[] = {0xE4, 0x65, 0xB8, 0x83, 0xD5, 0xA0}; // Replace with actual MAC address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  // Default channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register ESP-NOW send callback
  esp_now_register_send_cb(onDataSent);
}

void loop() {
  // Read ADC values
  float ntcReading = analogRead(NTC_PIN);
  float pressureReading = analogRead(MPX5700_PIN);

  // Calculate sensor values
  dataToSend.temperature = calculateTemperature(ntcReading);
  dataToSend.pressure = calculatePressure(pressureReading);

  // Send data via ESP-NOW
  esp_now_send(NULL, (uint8_t*)&dataToSend, sizeof(dataToSend));

  // Print sensor values to Serial Monitor for debugging
  Serial.print("Temperature: ");
  Serial.print(dataToSend.temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(dataToSend.pressure);
  Serial.println(" kPa");

  delay(2000);  // Update every 2 seconds
}
