#include <Wire.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);
const int buttonPin = 2;  // Button connected to digital pin 2

// Calibration offsets
float offsetX = 0, offsetY = 0, offsetZ = 0;

void setup() {
    Serial.begin(9600);
    pinMode(buttonPin, INPUT_PULLUP);  // Use internal pull-up resistor

    if (!accel.begin()) {
        Serial.println("Could not find ADXL345 sensor!");
        while (1);
    }

    Serial.println("Press the button to calibrate.");
}

void loop() {
    sensors_event_t event;
    accel.getEvent(&event);

    // Check if button is pressed (LOW because of pull-up resistor)
    if (digitalRead(buttonPin) == LOW) {
        calibrateSensor();
        Serial.println("Calibration done!");
        delay(1000); // Debounce delay
    }

    // Apply calibration offsets
    float calibratedX = event.acceleration.x - offsetX;
    float calibratedY = event.acceleration.y - offsetY;
    float calibratedZ = event.acceleration.z - offsetZ;

    Serial.print("X: "); Serial.print(calibratedX);
    Serial.print(" Y: "); Serial.print(calibratedY);
    Serial.print(" Z: "); Serial.println(calibratedZ);

    delay(1500);
}

void calibrateSensor() {
    sensors_event_t event;
    accel.getEvent(&event);

    // Store offsets (assuming the sensor should be level)
    offsetX = event.acceleration.x;
    offsetY = event.acceleration.y;
    offsetZ = event.acceleration.z - 9.81;  // Gravity compensation

    Serial.println("Calibration complete! Offsets stored.");
}
