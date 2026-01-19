#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// Pin Definitions
#define CHARGING_PIN 19
#define LED_PIN 2

// Battery Configuration
const float MIN_VOLTAGE = 13.0;           // Voltage at 0% battery
const float MAX_VOLTAGE = 16.8;           // Voltage at 100% battery
const float VOLTAGE_DIVIDER_RATIO = 5.1;  // From 41k & 10k resistors

// Measurement Timing
const unsigned long MEASUREMENT_INTERVAL = 2000;  // 3 seconds

// Bluetooth Configuration
BluetoothSerial solarGenerator;
const char *pin = "abcd";
String device_name = "PowerLabs 300W SolarGen";

// ADS1115 Setup
Adafruit_ADS1115 ads;  // Create ADS1115 object

// Global variables
float batteryVoltage = 0.0;
int batteryPercentage = 0;
bool isCharging = false;
bool previousChargingState = false;
unsigned long lastMeasurement = 0;

// Function Prototypes
float readBatteryVoltage();
int calculateBatteryPercentage(float voltage);
void checkChargingStatus();
void sendDataToApp();

void setup() {
  Serial.begin(115200);

  // Initialize I2C and ADS1115
  Wire.begin();
  // ads.begin(0x48);
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS1115. Check wiring!");
    while (1)
      ;
  }
  ads.setGain(GAIN_ONE);  // ±4.096V input range

  pinMode(CHARGING_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Start Bluetooth
  solarGenerator.begin(device_name);
  solarGenerator.setPin(pin);

  Serial.printf("The device \"%s\" is started.\nPair via Bluetooth to connect.\n", device_name.c_str());
}  // end setup()

void loop() {

  // Periodically measure battery and send data
  if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
    batteryVoltage = readBatteryVoltage();
    batteryPercentage = calculateBatteryPercentage(batteryVoltage);
    checkChargingStatus();

    sendDataToApp();  // send the data to connected app

    Serial.printf("Battery Voltage: %.2fV | Percentage: %d%% | Charging: %s\n",
                  batteryVoltage, batteryPercentage, isCharging ? "YES" : "NO");
    lastMeasurement = millis();
  }


}  // end main loop

float readBatteryVoltage() {
  // Read raw ADC value from ADS1115 channel A3
  int16_t raw = ads.readADC_SingleEnded(3);  // A3 channel

  // ADS1115 with GAIN_ONE: 1 bit = 0.125mV = 0.000125V
  float analogVoltage = raw * 0.000125;

  // Scale up using the voltage divider ratio
  float measuredVoltage = analogVoltage * VOLTAGE_DIVIDER_RATIO;

  Serial.printf("RAW: %d | A3: %.3f V | Battery calc: %.2f V\n",
                raw, analogVoltage, measuredVoltage);

  // Constrain to expected battery voltage range
  return constrain(measuredVoltage, MIN_VOLTAGE, MAX_VOLTAGE) - 0.02;
}  // end readBatteryVoltage()

int calculateBatteryPercentage(float voltage) {
  int percentage = (int)(((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0);
  return constrain(percentage, 0, 100);
}  // end calculateBatteryPercentage()

void checkChargingStatus() {
  bool currentChargingState = digitalRead(CHARGING_PIN);
  isCharging = currentChargingState;

  String statusMsg = currentChargingState ? "CHARGING_STARTED" : "CHARGING_STOPPED";
  Serial.println("[INFO] " + statusMsg);
}  // end checkChargingStatus()

void sendDataToApp() {
  String packet = " {\"Voltage\": " + String(batteryVoltage, 2)
                  + ", \"BatteryPercentage\": " + String(batteryPercentage)
                  + ", \"ChargingStatus\": " + (isCharging ? "true" : "false") + "}";
  solarGenerator.println(packet);
}  // end sendDataToApp()
