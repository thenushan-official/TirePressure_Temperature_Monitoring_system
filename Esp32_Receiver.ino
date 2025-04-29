#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define LED_PIN 2  

LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C LCD with address 0x27

typedef struct {
  float temperature;
  float pressure;
} SensorData;

SensorData receivedData;

// Callback function to receive data
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  // Blink the onboard LED to indicate data reception
  digitalWrite(LED_PIN, HIGH);  // Turn the LED on
  delay(100);                   // Keep it on for 100ms
  digitalWrite(LED_PIN, LOW);   // Turn the LED off

  // Display the received data on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.print(receivedData.temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Pressure:");
  lcd.print(receivedData.pressure);
  lcd.print("kPa");

  // Serial monitor output
  Serial.print("Temperature: ");
  Serial.print(receivedData.temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(receivedData.pressure);
  Serial.println(" Psi");

  // Show warnings if temperature or pressure exceeds thresholds
  if (receivedData.temperature > 30.0) {
    Serial.println("WARNING: Temperature Exceeded 30°C!");
    lcd.setCursor(0, 2);
    lcd.print("TEMPERATURE WARNING!");
  }

  if (receivedData.temperature < 30.0) {
    Serial.println("SAFE");
    lcd.setCursor(0, 2);
    lcd.print("TEMPERATURE NORMAL");
  }

  if (receivedData.pressure >= 35.0) {
    Serial.println("WARNING: Pressure Exceeded 35psi!");
    lcd.setCursor(0, 3);
    lcd.print("PRESSURE HIGH!");
  }

  if (receivedData.pressure <= 15.0) {
    Serial.println("WARNING: Pressure less than 15psi!");
    lcd.setCursor(0, 3);
    lcd.print("PRESSURE LOW!");
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  // Set onboard LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure the LED is off initially

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving data
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // The main loop remains empty as the data handling is done in the callback
}
