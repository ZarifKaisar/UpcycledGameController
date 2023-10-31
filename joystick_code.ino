/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-joystick
 */



#include <Mouse.h>

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 8;  // the number of the pushbutton pin
const int VibrationMotorPin = 9;    // the number of the LED pin


// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status


#include <Keyboard.h>
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

int xValue = 0 ; // To store value of the X axisawwwaww
int yValue = 0 ; // To store value of the Y axis
int command = COMMAND_NO;

void setup() {
  Serial.begin(9600) ;

    // initialize the LED pin as an output:
  pinMode(VibrationMotorPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

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
     //delay(50);
    // TODO: add your task here
  }

  if (command & COMMAND_RIGHT) {
    Keyboard.write('s');
   //delay(50);
    
    // TODO: add your task here
  }

  if (command & COMMAND_UP) {
    Keyboard.write('d');
     //delay(50);
    // TODO: add your task here
  }

  if (command & COMMAND_DOWN) {
    Keyboard.write('a');
     //delay(50);
    // TODO: add your task here
  }














    // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    Mouse.click();
    digitalWrite(VibrationMotorPin, HIGH);
    delay(100);
  } else {
    digitalWrite(VibrationMotorPin, LOW);
    delay(100);
  }
}
