//Range: 0 to 1000
//<500 = Road
//>600 = Sidewalk
//500-600 ignore due to shadows
//Use a flashlight or assortment of white LEDs to minimize shadow impact

#define IR_SENSOR_ANALOG A0  // Analog pin connected to TCRT5000 A0
#define IR_SENSOR_DIGITAL 2  // Digital pin connected to TCRT5000 D0 (optional)

void setup() {
    Serial.begin(9600);
    pinMode(IR_SENSOR_DIGITAL, INPUT);
}

void loop() {
    int analogValue = analogRead(IR_SENSOR_ANALOG);  // Read analog value
    int digitalValue = digitalRead(IR_SENSOR_DIGITAL);  // Read digital output (if available)

    Serial.print("Analog: ");
    Serial.print(analogValue);
    Serial.print(" | Digital: ");
    Serial.println(digitalValue);

    // Interpretation
    if (analogValue > 700) {
        Serial.println("High reflection detected (light surface)");
    } else if (analogValue < 300) {
        Serial.println("Low reflection detected (dark surface)");
    } else {
        Serial.println("Intermediate reflection (gray/surface variation)");
    }

    delay(500); // Small delay to avoid spamming the serial monitor
}
