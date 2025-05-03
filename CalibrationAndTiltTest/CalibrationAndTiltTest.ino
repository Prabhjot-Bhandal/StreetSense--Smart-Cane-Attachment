//Importing libraries
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>

//Defines the accelerometer as an accelerometer object in the code
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);

//Button to start the calibration
const int buttonPin = 2;

/*Variables to store the offset values for X, Y, Z, this is
done to create a "new" x-axis since we are taking the angle from the
x-axis.*/
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

//Tilt detection threshold in degrees
const float tiltThreshold = 5.0;

void setup() {
    Serial.begin(9600);
    pinMode(buttonPin, INPUT_PULLUP);

    //If the ADXL345 is not setup correctly
    if (!accel.begin()) {
        Serial.println("Could not find ADXL345 sensor");
        while (1);
    }

    /*Set the ADXL345 to +-2g mode
    0x31 is how the ADXL345 registers data, 0x00 sets the format to +-2g (higher sensitivity)*/
    accel.writeRegister(0x31, 0x00);

    //Calibrates the sensor to its current location upon startup
    calibrateSensor();
}

void loop() {
    //Check if the button is pressed (LOW because of pull-up resistor),
    //button pressed -> callibration
    if (digitalRead(buttonPin) == LOW) {
        calibrateSensor();
        Serial.println("Calibration done!");
        //Debounce delay, buttons can cause big and sudden changes in power
        delay(1000); 
    }

    //Sensor waits to be moved (the event), then gets the event
    sensors_event_t event;
    accel.getEvent(&event);

    //Calculate the angle relative to the X-axis after calibration
    //Multiplies by 180/PI since angles in C++ are in rad
    //Angle from x-axis, the y and z vectors are relative to the x vector
    float angleX = atan2(event.acceleration.y, event.acceleration.z) * (180.0 / PI);
    //Angle from y-axis, the x and z vectors are relative to the y vector
    float angleY = atan2(event.acceleration.x, event.acceleration.z) * (180.0 / PI);

    /*Calculate the relative tilt from the calibration offsets, angles from
    the "new" x and y axes*/
    float relativeAngleX = angleX - (atan2(offsetY, offsetZ) * (180.0 / PI));
    float relativeAngleY = angleY - (atan2(offsetX, offsetZ) * (180.0 / PI));

    //Print the tilt angles relative to the new axes
    Serial.print("Relative Tilt Angle X-axis: ");
    Serial.print(relativeAngleX);
    Serial.println("°");
    
    Serial.print("Relative Tilt Angle Y-axis: ");
    Serial.print(relativeAngleY);
    Serial.println("°");

    //Determine veering direction based on tilt
    //This really depends on how we orient the device on the cane
    //Can add a deadzone if needed
    if (relativeAngleX < -tiltThreshold) {
      Serial.println("Veering Right");
    } 
    else if (relativeAngleX > tiltThreshold) {
      Serial.println("Veering Left");
    } 
    else {
      Serial.println("Moving Straight");
    }

    //Delay between each reading
    delay(500);
}

void calibrateSensor() {
    Serial.println("Place the sensor in the desired orientation and press the button to calibrate.");
    //Allow time for the sensor to settle
    delay(2000);  

    //Waits for the button to be pressed
    sensors_event_t event;
    accel.getEvent(&event);

    //Print the current raw accelerometer values
    Serial.print("Raw X: ");
    Serial.print(event.acceleration.x);
    Serial.print(" Y: ");
    Serial.print(event.acceleration.y);
    Serial.print(" Z: ");
    Serial.println(event.acceleration.z);

    //Store the current accelerometer values as the calibration reference
    //This allows us to make a new x and y axis
    offsetX = event.acceleration.x;
    offsetY = event.acceleration.y;
    offsetZ = event.acceleration.z;

    //Print the calibration offsets
    Serial.print("Calibration offsets -> X: ");
    Serial.print(offsetX);
    Serial.print(" Y: ");
    Serial.print(offsetY);
    Serial.print(" Z: ");
    Serial.println(offsetZ);

    //After calibration, immediately display the angle (should be close to 0° if aligned)
    float angle = atan2(offsetY, offsetZ) * (180.0 / PI);
    Serial.print("Tilt Angle after calibration (should be close to 0°): ");
    Serial.println(angle);

    Serial.println("Calibration complete! Keep the sensor still.");

} //calibrateSensor()
