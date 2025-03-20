# Heal-monitoring-system
/**
 * Health Monitoring System
 * Sensors:
 * - Heart Rate Sensor (XD-58C)
 * - Body Temperature Sensor (DS18B20)
 * - Accelerometer/Gyroscope (ADXL335)
 * - Pulse Oximeter (MAX30100)
 */

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <MAX30100_PulseOximeter.h>

// Pin definitions
#define TEMP_SENSOR_PIN 2       // DS18B20 Temperature sensor pin
#define HEARTBEAT_SENSOR_PIN A0 // XD-58C analog pin
#define ACCEL_X_PIN A1          // ADXL335 X-axis pin
#define ACCEL_Y_PIN A2          // ADXL335 Y-axis pin
#define ACCEL_Z_PIN A3          // ADXL335 Z-axis pin

// Constants
#define REPORTING_PERIOD_MS 1000 // Update interval in milliseconds

// Global variables
unsigned long lastReportTime = 0;
float bodyTemperature = 0.0;
int heartRate = 0;
int spO2 = 0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
bool heartBeatDetected = false;

// Initialize libraries
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20x4 LCD
PulseOximeter pox;

// Callback for pulse oximeter
void onBeatDetected() {
  heartBeatDetected = true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Health Monitoring System Initializing...");
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Health Monitor");
  delay(2000);
  
  // Initialize pulse oximeter
  Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    lcd.setCursor(0, 1);
    lcd.print("Pulse Ox Error!");
    while (1);
  } else {
    Serial.println("SUCCESS");
  }
  
  // Configure pulse oximeter
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Update pulse oximeter readings
  pox.update();
  
  // Read temperature sensor
  tempSensor.requestTemperatures();
  bodyTemperature = tempSensor.getTempCByIndex(0);
  
  // Read heart rate sensor (XD-58C)
  int rawHeartbeat = analogRead(HEARTBEAT_SENSOR_PIN);
  
  // Read accelerometer values
  accelX = mapAccelValue(analogRead(ACCEL_X_PIN));
  accelY = mapAccelValue(analogRead(ACCEL_Y_PIN));
  accelZ = mapAccelValue(analogRead(ACCEL_Z_PIN));
  
  // Report data at fixed intervals
  if (millis() - lastReportTime > REPORTING_PERIOD_MS) {
    // Get SpO2 and heart rate values from pulse oximeter
    spO2 = pox.getSpO2();
    
    // Use pulse oximeter heart rate or XD-58C based on availability
    if (pox.getHeartRate() > 0) {
      heartRate = pox.getHeartRate();
    } else {
      // Simple heart rate calculation from XD-58C
      // This is a basic implementation and would need to be refined
      heartRate = map(rawHeartbeat, 0, 1023, 40, 180);
    }
    
    // Display values on LCD
    updateLCD();
    
    // Output to serial monitor for debugging
    printToSerial();
    
    lastReportTime = millis();
    heartBeatDetected = false;
  }
}

float mapAccelValue(int rawValue) {
  // Convert analog reading (0-1023) to acceleration in g (-3g to +3g)
  // ADXL335 typically has a sensitivity of about 330mV/g at 3.3V
  // Zero g offset is at about 1.5V or ~512 analog reading
  return (rawValue - 512) / 102.3; // 1g is about 102.3 units
}

void updateLCD() {
  lcd.clear();
  
  // Row 1: Temperature and Heart Rate
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(bodyTemperature);
  lcd.print("C");
  
  lcd.setCursor(11, 0);
  lcd.print("HR:");
  lcd.print(heartRate);
  lcd.print("bpm");
  
  // Row 2: SpO2
  lcd.setCursor(0, 1);
  lcd.print("SpO2:");
  lcd.print(spO2);
