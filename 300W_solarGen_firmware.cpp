#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// Pin Definitions
#define VOLTAGE_PIN 35
#define CHARGING_PIN 19
#define LED_PIN 2

// Battery Configuration
const float MIN_VOLTAGE = 13.0;  // Voltage at 0%
const float MAX_VOLTAGE = 16.8;  // Voltage at 100%
const int ADC_RESOLUTION = 4095;
const float REFERENCE_VOLTAGE = 3.3;

// Alert Threshold
const int LOW_BATTERY_THRESHOLD = 20;
const unsigned long ALERT_COOLDOWN = 10000;  // 10 seconds

// Measurement Interval
const unsigned long MEASUREMENT_INTERVAL = 5000;

// Bluetooth Configuration
BluetoothSerial solarGenerator;
const char *pin = "abcd";
String device_name = "PowerLabs Solar Generator";

// Globals
float batteryVoltage = 0.0;
int batteryPercentage = 0;
bool isCharging = false;
bool previousChargingState = false;
bool lowBatteryAlertSent = false;
unsigned long lastMeasurement = 0;
unsigned long lastLowBatteryAlert = 0;

// Measurement Smoothing variable
const int AVERAGE_WINDOW = 10;
int adcSamples[AVERAGE_WINDOW] = { 0 };
int sampleIndex = 0;
bool bufferFilled = false;

// Exponential Moving Average smoothing
float smoothedVoltage = 0.0;
const float EMA_ALPHA = 0.2;  // Smoothing factor;
bool firstVoltageRead = true;

// Function Prototypes
void measureVoltage();
void checkChargingStatus();
void checkBatteryAlert();
void sendAlert(const String &type, int percentage);
void sendDataToApp();
void handleBluetoothCommand(const String &command);

void setup() {

  Serial.begin(115200);

  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CHARGING_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  solarGenerator.begin(device_name);
  solarGenerator.setPin(pin);

  Serial.printf("The device \"%s\" is started.\nPair via Bluetooth to connect.\n", device_name.c_str());
}

void loop() {
  // Check charging status and notify if changed
  checkChargingStatus();

  // Always update battery logic
  measureVoltage();

  // Send periodic data + debug info
  if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
    // Send data to App
    sendDataToApp();
    Serial.printf("Battery Voltage: %.2fV | Percentage: %d%% | Charging: %s\n",
                  batteryVoltage, batteryPercentage, isCharging ? "YES" : "NO");
    lastMeasurement = millis();
  }
}

// Measure Battery Voltage and Percentage
void measureVoltage() {
  int rawADC = analogRead(VOLTAGE_PIN);
  float analogVoltage = (rawADC / (float)ADC_RESOLUTION) * REFERENCE_VOLTAGE;
  float measuredVoltage = (analogVoltage / REFERENCE_VOLTAGE) * MAX_VOLTAGE;
  measuredVoltage = constrain(measuredVoltage, MIN_VOLTAGE, MAX_VOLTAGE);

  // Initialize EMA on first run
  if (firstVoltageRead) {
    smoothedVoltage = measuredVoltage;
    firstVoltageRead = false;
  } else {
    smoothedVoltage = EMA_ALPHA * measuredVoltage + (1.0 - EMA_ALPHA) * smoothedVoltage;
  }

  batteryVoltage = smoothedVoltage;

  batteryPercentage = (int)(((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
}

// Check Charging State and Notify if Changed
void checkChargingStatus() {
  bool currentChargingState = !digitalRead(CHARGING_PIN);  // Active low

  // Send charging status message
  if (currentChargingState != previousChargingState) {
    previousChargingState = currentChargingState;
    isCharging = currentChargingState;

    String statusMsg = currentChargingState ? "CHARGING_STARTED" : "CHARGING_STOPPED";
    // solarGenerator.println(statusMsg);
    Serial.println("[INFO] " + statusMsg);
    sendDataToApp();
  }

  else {
    isCharging = currentChargingState;
  }
  // Blink LED if charging
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (isCharging && millis() - lastBlink >= 200) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }

  else if (!isCharging) {
    digitalWrite(LED_PIN, LOW);
  }
}

// Send Data Packet to App
void sendDataToApp() {
  String packet = " {\"Voltage\": " + String(batteryVoltage, 2)
                  + ", \"BatteryPercentage\": " + String(batteryPercentage)
                  + ", \"ChargingStatus\": " + (isCharging ? "true" : "false") + "}";
  solarGenerator.println(packet);
}
