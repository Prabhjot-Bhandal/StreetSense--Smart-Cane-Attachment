//GROUP; Mon-23
//TITLE: 1P13 P3: Refined Crosswalk Guide Prototype

/*TO DO:
    DONE: 0. Reorganize existing code into separate functions.
    
    DONE: 1. Implement the IR sensor.  
    DONE: 1.1. Change contrast so that it detects for high contrast,
       and not low (the road is low, so we don't want her to
       start walking on the road.)
    DONE: 1.2. Wire the IR sensor correctly
    DONE: 1.3. Test the IR sensor with diagnostic code.
         
    DONE: 2. Implement vibration motor.
    DONE: 2.1. Interface the vibration motor correctly.
    DONE: 2.2. Create code to give output to the vibration motor.
    DONE: 2.3. Create specific functions in the code to be called on
         to tell the user to go to left or right using the vibra-
         tion motors.
    
        
    DONE: 3. Implement the accelerometer.
    DONE: 3.1. Interface the accelerometer into the circuit.
    DONE: 3.2. Create a function to check for acceleration/speed.
    DONE: 3.4. Code gives feedback to the user using the vibration motor
         functions.
    DONE: 3.5. Test the code to see if the accelerometer and the
         vibration feedback works.
    
    DONE: 4. Interface the circuit in REAL LIFE based off of the TinkerCAD (doesn't include ADXL345)
    DONE: 5. Attach the circuit to a stick with the makeshift casing.
    DONE: 6. Test overall. Make adjustments to the code when needed.
    
*/

//IMPORTING LIBRARIES
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>

//PIN VARIABLES
//const int's are so that the variables can't be changed
//BUTTONS + LEDS
const int START_BUTTON = 0; //PURPLE
const int CROSSWALK_DETECTED_LED = 2; //ORANGE
const int IR_CALIBRATION_BUTTON = 3; //BLUE
const int NOT_ON_CROSSWALK_LED = 4; //BLUE
const int ACCEL_CALIBRATION_BUTTON = 6; //PINK

//VIBRATION MOTOR
const int VIBRATION_PIN = 5; //GREEN

//IR SENSOR
const int IR_SENSOR_PIN = A0; //LIGHT BLUE

//IR SENSOR THRESHOLD
int irThreshold = 0; //Initial value of the IR threshold

//ACCELEROMETER, Defines the accelerometer as an accelerometer object in the code
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123); //GREY

//ACCELEROMETER CALIBRATION LOGIC VARIABLES
/*Variables to store the offset values for X, Y, Z, this is
done to create a "new" x-axis since we are taking the angle from the
x-axis.*/
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;
const float tiltThreshold = 10.0; //Tilt detection threshold in degrees
float relativeAngleX = 0.0; //For checking if user is past angle threshold

//CROSSWALK LOGIC VARIABLES
unsigned long lastIndentTime = 0;
//Removed velocity calculations, cannot get accurate velocity with accelerometer
const int timeIntervalStrips = 4000; //Upper limit on time for when crosswalk not found

//PUSHBUTTON TRACKING
//Stores the previous state of the button (default HIGH due to INPUT_PULLUP)
bool lastButtonState = HIGH; 
//Tracks whether the device is on or off
bool deviceOn = false; 

void setup() {
  //Inputs; Sensors + Switches
  pinMode(START_BUTTON, INPUT_PULLUP); //Using INPUT_PULLUP to avoid floating state
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(ACCEL_CALIBRATION_BUTTON, INPUT_PULLUP); //Calibration button pin for accel
  pinMode(IR_CALIBRATION_BUTTON, INPUT_PULLUP); //Calibration button pin for IR
  
  //Accelerometer Interfacing Verification, stops code if not properly interfaced
  if (!accel.begin()) {
    Serial.println("Could not find ADXL345 sensor");
    while (1);
  }

  //Accelerometer Sensitivity Setup: Set the ADXL345 to +-2g mode
  accel.writeRegister(0x31, 0x00);

  //Outputs; Motors + LEDS
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(CROSSWALK_DETECTED_LED, OUTPUT);
  pinMode(NOT_ON_CROSSWALK_LED, OUTPUT);
  
  //Starts serial monitor
  Serial.begin(9600);

  //Calibration of other components
  calibrateIRSensor();
  calibrateAccel();
  
  //Tests the vibration motor + tells user if circuit is plugged in and
  //program has been started
  Serial.println("VIBRATION ON");
  digitalWrite(VIBRATION_PIN, HIGH); //Ensures that the vibration motor is low after the test
  delay(2000);
  Serial.println("VIBRATION OFF");
  digitalWrite(VIBRATION_PIN, LOW); //Ensures that the vibration motor is low after the test

} //setup()

//Function to start the program logic when the button is pressed
void startProgram() {
  unsigned long currentTime = millis();

  //ACCELEROMETER FUNCTION, tracks user's tilt
  runAccel();
  
  //IR SENSOR FUNCTION
  //Accel and Vibrate are ran in this
  IRSensor(currentTime, timeIntervalStrips);

} //startProgram()

void loop() {
  //Checks the button state (HIGH or LOW)
  bool buttonState = digitalRead(START_BUTTON);
  
  //Detects when button is *pressed* (LOW to HIGH transition)
  //Causes turning OFF system to hold down the pushbutton
  if (buttonState == LOW && lastButtonState == HIGH) {
    Serial.println("Button Pressed!");

    //Toggle the device on/off
    //ON
    if (deviceOn) {
      Serial.println("Turning device OFF.");
      //When turning OFF the device
      digitalWrite(CROSSWALK_DETECTED_LED, LOW);
      digitalWrite(NOT_ON_CROSSWALK_LED, LOW);
      digitalWrite(VIBRATION_PIN, LOW);
      deviceOn = false;
    } 
    //OFF
    else {
      Serial.println("Turning device ON.");

      //Turn ON the device
      deviceOn = true;
    }
  }

  //If the device is ON, run the program
  if (deviceOn) {
    startProgram();
  }

  //Store the button state for next loop iteration
  lastButtonState = buttonState;

  delay(50); //Debounce delay to prevent multiple triggers
  
} //loop()


//IR SENSOR
void IRSensor(unsigned long currentTime, double timeInterval) {
  //Check if the button is pressed (LOW because of pull-up resistor),
  //button pressed -> callibration
  if (digitalRead(IR_CALIBRATION_BUTTON) == LOW) {
      Serial.println("");
      calibrateIRSensor();
      Serial.println("IR calibration done!");
      Serial.println("");
      //Debounce delay, buttons can cause big and sudden changes in power
      delay(1000); 
  }

  //Reads the IR Sensor
  int sensorValue = analogRead(IR_SENSOR_PIN);
  //Prints out the current IR sensor reading
  Serial.println("IR Sensor Contrast: " + String(sensorValue));
  
  //When crosswalk strip detected (low contrast??, should be high but from testing it is low)
  //If we need to hardcode, <100 gives the most accurate readings of white/high contrast
  //Below 30 was white, Above 30 was the pavement?
  if (sensorValue < irThreshold) {
    Serial.println("Strip Detected; Still on Crosswalk");
    //Resets timer for last strip seen
    lastIndentTime = currentTime;
    digitalWrite(CROSSWALK_DETECTED_LED, HIGH);
    digitalWrite(NOT_ON_CROSSWALK_LED, LOW);
  } 
  //When haven't detected a crosswalk strip for awhile
  else if ((currentTime - lastIndentTime) > timeInterval) {
    digitalWrite(NOT_ON_CROSSWALK_LED, HIGH);
    Serial.println("NOT ON CROSSWALK!");
    //Uses global variable "relativeAngleX" calculated by runAccel() to check
    //if user is going left or right
    checkAccelThreshold(); //decideVibration() is called in this function
  } 
  //When not on crosswalk strip
  else {
    digitalWrite(CROSSWALK_DETECTED_LED, LOW);
    Serial.println("Away from crosswalk strip.");
  }
  
} //IRSensor()

void calibrateIRSensor() {
//WARNING: Can only be calibrated when the IR sensor faces HIGH-CONTRAST!!

  //Sums up all current IR sensor values and then finds the average of them
  int sum = 0;
  //Keeps checking for readings
  for (int i = 0; i < 10; i++) {
    sum += analogRead(IR_SENSOR_PIN);
    delay(50);
  }

  //Changes global variable irThreshold with the new average
  irThreshold = sum / 10 + 50; // Set threshold slightly above average

  //Prints out the calibrated threshold
  Serial.print("Calibrated IR Threshold: ");
  Serial.println(irThreshold);

} //calibrateIRSensor()


//ACCELEROMETER SENSOR
void runAccel() {
  //Check if the button is pressed (LOW because of pull-up resistor),
    //button pressed -> callibration
    if (digitalRead(ACCEL_CALIBRATION_BUTTON) == LOW) {
        Serial.println("");
        calibrateAccel();
        Serial.println("Accel calibration done!");
        Serial.println("");
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
    relativeAngleX = angleX - (atan2(offsetY, offsetZ) * (180.0 / PI));
    float relativeAngleY = angleY - (atan2(offsetX, offsetZ) * (180.0 / PI));

    //Print the tilt angles relative to the new axes
    Serial.print("Relative Tilt Angle X-axis: ");
    Serial.print(relativeAngleX);
    Serial.println("°");
    
    Serial.print("Relative Tilt Angle Y-axis: ");
    Serial.print(relativeAngleY);
    Serial.println("°");

} //runAccel()

void checkAccelThreshold() {
  //Runs if user off the crosswalk, checks to see if the user is going
  //left or right. runAccel needs to always run for vector tracking.

  //Determine veering direction based on tilt
    //This really depends on how we orient the device on the cane
    //RIGHT
    if (relativeAngleX < -tiltThreshold) {
      Serial.println("Veering Right");
      //If veering right, tell user to go left (1 vibration)
      decideVibration(1);
    } 
    //LEFT
    else if (relativeAngleX > tiltThreshold) {
      //If veering left, tell user to go right, (2 vibrations)
      Serial.println("Veering Left");
      decideVibration(-1);
    } 
    //STRAIGHT
    else {
      Serial.println("Moving Straight");
      decideVibration(0);
    }

  //Delay between each reading, need this or else
  //the accel begins overcorrecting itself
  delay(500);

} //checkAccelThreshold()

void calibrateAccel() {
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

} //calibrateAccel()


//VIBRATION MOTOR
void vibrate(int times) {
  //To show that vibration should be occurring
  Serial.print("Vibrating ");
  Serial.print(times);
  Serial.println(" time(s)");
  
  /*Loop used to differentiate left (2 vibrations) and 
  right (1 vibration) vibrations, this is done since the 
  entire device runs within a loop*/
  for (int i = 0; i < times; i++) {
    digitalWrite(VIBRATION_PIN, HIGH);
    delay(300);
    digitalWrite(VIBRATION_PIN, LOW);
    delay(300);
  }
  
} //vibrate()

//Decides vibration feedback based on accelerometer reading
void decideVibration(int leftOrRight) {
  //GOING LEFT, GUIDE TO RIGHT
  if (leftOrRight < 0) {
    vibrate(2); //Vibrate twice for right
  }
  //GOING RIGHT, GUIDE TO LEFT
  else if (leftOrRight > 0) {
    vibrate(1); //Vibrate once for left
  } 
  //STRAIGHT
  else {
    Serial.println("Moving straight, no vibration.");
    digitalWrite(VIBRATION_PIN, LOW); //Ensures that the vibration motor is low
  }
  
} //decideVibration
