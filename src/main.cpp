#include <Arduino.h>
#include <AccelStepper.h>

/*
  JoystickMouseControl

  Controls the mouse from a joystick on an Arduino Leonardo, Micro or Due.
  Uses a pushbutton to turn on and off mouse control, and a second pushbutton
  to click the left mouse button.

  Hardware:
  - 2-axis joystick connected to pins A0 and A1
  - pushbuttons connected to pin D2 and D3

  The mouse movement is always relative. This sketch reads two analog inputs
  that range from 0 to 1023 (or less on either end) and translates them into
  ranges of -6 to 6.
  The sketch assumes that the joystick resting values are around the middle of
  the range, but that they vary within a threshold.

  WARNING: When you use the Mouse.move() command, the Arduino takes over your
  mouse! Make sure you have control before you use the command. This sketch
  includes a pushbutton to toggle the mouse control state, so you can turn on
  and off mouse control.

  created 15 Sep 2011
  updated 28 Mar 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/JoystickMouseControl
*/

int readAxis(int thisAxis);
void setSpeed(int speed);
void driveMotor(bool CW);


// set pin numbers for switch, joystick axes, and LED:
const int switchPin = 2;      // switch to turn on and off mouse control
const int mouseButton = 3;    // input pin for the mouse pushButton
const int xAxis = A0;         // joystick X axis
const int yAxis = A1;         // joystick Y axis
const int ledPin = 5;         // Mouse control LED

// parameters for reading the joystick:
int range = 256;               // output range of X or Y movement
int responseDelay = 100;        // response delay of the mouse, in ms
//int threshold = range / 4;    // resting threshold
int threshold = 2;    // resting threshold
//int center = 522;       // resting position value

bool mouseIsActive = false;    // whether or not to control the mouse
int lastSwitchState = LOW;        // previous switch state

//AccelStepper stepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);

#define step_pin 2 // Pin 3 connected to Steps pin on EasyDriver
#define dir_pin 3 // Pin 2 connected to Direction pin
#define MS1 4 // Pin 5 connected to MS1 pin
#define MS2 5 // Pin 4 connected to MS2 pin
#define SLEEP 7 // Pin 7 connected to SLEEP pin



void setup() {
  pinMode(switchPin, INPUT);       // the switch pin
  pinMode(ledPin, OUTPUT);         // the LED pin

  // take control of the mouse:
//  Mouse.begin();

//  stepper.setMaxSpeed(100);
//  stepper.setAcceleration(20);

	Serial.begin(115200);

   pinMode (MS1, OUTPUT);
   pinMode (MS2, OUTPUT);
   pinMode (dir_pin, OUTPUT);
   pinMode (step_pin, OUTPUT);
   pinMode (SLEEP, OUTPUT);
   
   digitalWrite (SLEEP, HIGH); // Wake up EasyDriver
   delay (5); // Wait for EasyDriver wake up

   digitalWrite (MS1, LOW); // Configures to Full Steps
   digitalWrite (MS2, LOW); // Configures to Full Steps

}

void loop() {
  // read the switch:
  int switchState = digitalRead(switchPin);
  // if it's changed and it's high, toggle the mouse state:
  if (switchState != lastSwitchState) {
    if (switchState == HIGH) {
      mouseIsActive = !mouseIsActive;
      // turn on LED to indicate mouse state:
      digitalWrite(ledPin, mouseIsActive);
    }
  }
  // save switch state for next comparison:
  lastSwitchState = switchState;

  // read and scale the two axes:
//  int xReading = readAxis(A0);
  int yReading = readAxis(A1);

	Serial.println(String(yReading));

  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    //Mouse.move(xReading, yReading, 0);
  }

  // read the mouse button and click or not click:
  // if the mouse button is pressed:

	int mvs = abs(yReading);

	if (mvs>900)
		setSpeed(0);
	else if (mvs>600)
		setSpeed(1);
	else if (mvs>400)
		setSpeed(2);
	else
		setSpeed(3);

  	if (yReading>50) 
  	{
		driveMotor(true);
  	}
  	else if(yReading<-50) 
	{
		driveMotor(false);
  	}
  	else
  	{
	  	//stepper.stop();
		digitalWrite (SLEEP, LOW);
	}

//stepper.run();

//  delay(responseDelay);
//	delay (1);
}

/*
  reads an axis (0 or 1 for x or y) and scales the analog input range to a range
  from 0 to <range>
*/

int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);
  //int center = 522;
  int delta = 11;

	//reading -= delta;

  // map the reading from the analog input range to the output range:
  	reading = map(reading-delta, 0, 1023, -1100, 1100);
	reading = max(min(reading, 1000), -1000);

	if(reading>-10 && reading<10)
		reading = 0;

  // if the output reading is outside from the rest position threshold, use it:
	return reading;

/*
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
  */
}

void setSpeed(int speed)
{
	switch (speed)
	{
		case 0:
			digitalWrite (MS1, LOW); // Configures to Full Steps
			digitalWrite (MS2, LOW); // Configures to Full Steps
			break;
		case 1:
			digitalWrite (MS1, HIGH); // Configures to Full Steps
			digitalWrite (MS2, LOW); // Configures to Full Steps
			break;
		case 2:
			digitalWrite (MS1, LOW); // Configures to Full Steps
			digitalWrite (MS2, HIGH); // Configures to Full Steps
			break;
		case 3:
			digitalWrite (MS1, HIGH); // Configures to Full Steps
			digitalWrite (MS2, HIGH); // Configures to Full Steps
			break;
	}
}

void driveMotor(bool CW)
{
	digitalWrite (SLEEP, HIGH);
	digitalWrite (dir_pin, (CW?LOW:HIGH)); // (HIGH = anti-clockwise / LOW = clockwise)
	digitalWrite (step_pin, HIGH);
	delay (1);
	digitalWrite (step_pin, LOW);
	delay (1);
}