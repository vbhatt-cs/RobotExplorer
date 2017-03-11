/*
 Name:		V1_MoveInGivenDirection.ino
 Created:	3/10/2017 9:08:58 AM
 Author:	Varun Bhatt
*/

#include <PID_v1.h>				//Arduino PID library
#include <Wire.h>				//I2C library
#include <FaBo9Axis_MPU9250.h>	//Library for MPU9250 reading
#include <SoftwareSerial.h>		//For communication with ESP8266
#include "ESP8266.h"

//Set to 1 if debugging in serial
const bool DEBUG_SERIAL = 1;

// Pins used on Arduino Uno
#define LPWM 10
#define LDIR1 8
#define LDIR2 9

#define RPWM 11
#define RDIR1 4
#define RDIR2 5

FaBo9Axis fabo_9axis;	// For MPU9250 readings

// Left and Right steady state speed
const byte RIGHT_SS_SPEED = 75;
const byte LEFT_SS_SPEED = 75;

// Left and Right speed
byte rightSpeed = 75;
byte leftSpeed = 75;

//PID Values
double inputx, inputy, inputz, output;

// PID Multipliers
double kp = 0.2;
double ki = 0;
double kd = 0.05;

double setDirection = 0;	//Reference direction
int setDistance = 100;		//Reference distance

int curDistance = 0;
bool runMotor = true;

//Setup PID
PID wheelControl(&inputx, &output, &setDirection, kp, ki, kd, DIRECT);

//Setup ESP8266
ESP8266 esp(10, 11, DEBUG_SERIAL);

void setup() {
	if (DEBUG_SERIAL)
		Serial.begin(9600);

	//pinMode(rEncoder, INPUT);
	//pinMode(lEncoder, INPUT);

	pinMode(RPWM, OUTPUT);
	pinMode(RDIR1, OUTPUT);
	pinMode(RDIR2, OUTPUT);

	pinMode(LPWM, OUTPUT);
	pinMode(LDIR1, OUTPUT);
	pinMode(LDIR2, OUTPUT);

	//Stop the motor initially
	digitalWrite(LDIR1, LOW);
	digitalWrite(LDIR2, LOW);
	analogWrite(LPWM, 0);
	digitalWrite(RDIR1, LOW);
	digitalWrite(RDIR2, LOW);
	analogWrite(LPWM, 0);

	//Initialize MPU9250
	fabo_9axis.begin();

	//Initialize ESP8266
	esp.init();

	//turn the PID on
	wheelControl.SetMode(AUTOMATIC);
}

void loop() {

	//Start the motor
	if (runMotor)
	{
		digitalWrite(LDIR2, LOW);
		digitalWrite(LDIR1, HIGH);
		digitalWrite(RDIR2, LOW);
		digitalWrite(RDIR1, HIGH);
	}

	//Read MPU9250
	fabo_9axis.readMagnetXYZ((float*)&inputx, (float*)&inputy, (float*)&inputz);

	//Get the direction and distance from webpage
	esp.getData(setDirection, setDistance);

	runMotor = curDistance < setDistance;
	curDistance = (curDistance == setDistance) ? 0 : curDistance;

	//Compute the PID output
	wheelControl.Compute();

	if (DEBUG_SERIAL)
		Serial.println(output);

	//Set the PWM values for the motor
	//
	//Output added to one wheel and subtracted from another wheel to rotate the bot
	//Change the sign of output (if needed) to make the bot rotate in a direction which reduces the error. 
	//
	//The motor was not moving for PWM below 50 and moving too fast for PWM over 100
	//So constrain the value between 50, 100
	rightSpeed = RIGHT_SS_SPEED - output;
	rightSpeed = constrain(rightSpeed, 50, 100);

	leftSpeed = LEFT_SS_SPEED + output;
	leftSpeed = constrain(leftSpeed, 50, 100);

	analogWrite(RPWM, rightSpeed);
	analogWrite(LPWM, leftSpeed);

	delay(100);
}