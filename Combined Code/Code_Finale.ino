#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>
#include <Keyboard.h>

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status

const int buttonPin = 8;  // tdshe number of the pushbutton pin
const int VibrationMotorPin = 9;    // the number of the LED pin

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy; 
#define VRX_PIN  A0 // Arduino pin connected to VRX pin
#define VRY_PIN  A1 // Arduino pin connected to VRY pin

#define LEFT_THRESHOLD  400
#define RIGHT_THRESHOLD 800
#define UP_THRESHOLD    400
#define DOWN_THRESHOLD  800

#define COMMAND_NO     0x00
#define COMMAND_LEFT   0x01
#define COMMAND_RIGHT  0x02
#define COMMAND_UP     0x04
#define COMMAND_DOWN   0x08

int xValue = 0 ; // To store value of the X axis
int yValue = 0 ; // To store value of the Y axis
int command = COMMAND_NO;

bool isJoystickIdle() {
  return (xValue >= LEFT_THRESHOLD && xValue <= RIGHT_THRESHOLD &&
          yValue >= UP_THRESHOLD && yValue <= DOWN_THRESHOLD);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) { while (1); }
    // initialize the LED pin as an output:
  pinMode(VibrationMotorPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // Read joystick data
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);
  buttonState = digitalRead(buttonPin);

  // If joystick is idle, read data from accelerometer and gyroscope
  if (isJoystickIdle() && buttonState == LOW) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // vx = (gx+305)/150;  // change 300 from 0 to 355
    // vy = -(gz-95)/150; // same here about "-100"  from -355 to 0

  vx = (gx+250)/150;  
  vy = -(gz-98)/150;

    // Serial.print("gx = ");
    // Serial.print(gx);
    // Serial.print(" | gz = ");
    // Serial.print(gz);
  
    // Serial.print("        | X = ");
    // Serial.print(vx);
    // Serial.print(" | Y = ");
    // Serial.println(vy);
  
    Mouse.move(vx, vy);
    digitalWrite(VibrationMotorPin, LOW);
  
    delay(20);
  }

  
if (buttonState == HIGH) {
    Mouse.click();
    digitalWrite(VibrationMotorPin, HIGH);
    delay(50);
  }
  // Joystick code

  // converts the analog value to commands
  // reset commands
  command = COMMAND_NO;

  // check left/right commands
  if (xValue < LEFT_THRESHOLD)
    command = command | COMMAND_LEFT;
  else if (xValue > RIGHT_THRESHOLD)
    command = command | COMMAND_RIGHT;

  // check up/down commands
  if (yValue < UP_THRESHOLD)
    command = command | COMMAND_UP;
  else if (yValue > DOWN_THRESHOLD)
    command = command | COMMAND_DOWN;

  // NOTE: AT A TIME, THERE MAY BE NO COMMAND, ONE COMMAND OR TWO COMMANDS

  // print command to serial and process command
  if (command & COMMAND_LEFT) {
    Keyboard.write('w');
    
    // TODO: add your task here
  }

  if (command & COMMAND_RIGHT) {
    Keyboard.write('s');
    
    // TODO: add your task here
  }

  if (command & COMMAND_UP) {
    Keyboard.write('d');
    
    // TODO: add your task here
  }

  if (command & COMMAND_DOWN) {
    Keyboard.write('a');
    
    // TODO: add your task here
  }
}
