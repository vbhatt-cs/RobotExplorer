/*
 Name:		V1_1_MoveInGivenDirectionWemos.ino
 Created:	3/11/2017 11:35:00 AM
 Author:	Varun Bhatt
*/

#include <PID_v1.h>				//Arduino PID library
//For magnetometer
#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
//For web server
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

//Set to 1 if debugging in serial
const bool DEBUG_SERIAL = 1;

// Pins used on Wemos D1 R2
#define LPWM D8
#define LDIR1 D7
#define LDIR2 D6

#define RPWM D5
#define RDIR1 D4
#define RDIR2 D3

#define ENCODER D0

// Left and Right steady state speed
const int RIGHT_SS_SPEED = 300;
const int LEFT_SS_SPEED = 300;
const int CONSTRAIN_LOW = 250;
const int CONSTRAIN_HIGH = 350;

// Left and Right speed
int rightSpeed = 300;
int leftSpeed = 300;

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

FaBo9Axis fabo_9axis;	// For MPU9250 readings

//Setup PID
PID wheelControl(&inputx, &output, &setDirection, kp, ki, kd, DIRECT);

//SSID of access point
const char *SSID = "WemosMotor";
ESP8266WebServer server(80);	//Server
String webpage;

void setup() {
	if (DEBUG_SERIAL)
		Serial.begin(9600);

	pinMode(ENCODER, INPUT);

	pinMode(RPWM, OUTPUT);
	pinMode(RDIR1, OUTPUT);
	pinMode(RDIR2, OUTPUT);

	pinMode(LPWM, OUTPUT);
	pinMode(LDIR1, OUTPUT);
	pinMode(LDIR2, OUTPUT);

	//Webpage to be displayed
	webpage = "<html><form method=\"get\">\n";
	webpage += "Direction (angle): <input type=\"number\" name=\"direction\" value=0 min=-180 max=180 step=0.1><br>\n";
	webpage += "Distance (encoder count): <input type=\"number\" name=\"distance\" value=0 min=0><br><br>\n";
	webpage += "<input type=\"submit\" value=\"Submit\">\n";
	webpage += "</form></html>";

	//Start the WiFi access point
	if (DEBUG_SERIAL)
	{
		Serial.println();
		Serial.print("Configuring access point...");
	}

	WiFi.softAP(SSID);
	IPAddress myIP = WiFi.softAPIP();

	if (DEBUG_SERIAL)
	{
		Serial.print("AP IP address: ");
		Serial.println(myIP);
	}

	server.on("/", handleRoot);
	server.begin();

	if (DEBUG_SERIAL)
		Serial.println("HTTP server started");

	//Stop the motor initially
	digitalWrite(LDIR1, LOW);
	digitalWrite(LDIR2, LOW);
	analogWrite(LPWM, 0);
	digitalWrite(RDIR1, LOW);
	digitalWrite(RDIR2, LOW);
	analogWrite(LPWM, 0);

	//Initialize MPU9250
	fabo_9axis.begin();

	//turn the PID on
	wheelControl.SetMode(AUTOMATIC);
}

void loop() {
	server.handleClient();	//Handle web requests

	//Read MPU9250
	fabo_9axis.readMagnetXYZ((float*)&inputx, (float*)&inputy, (float*)&inputz);

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
	//The motor was not moving for PWM below CONSTRAIN_LOW and moving too fast for PWM over CONSTRAIN_HIGH
	//So constrain the value inside that range
	rightSpeed = RIGHT_SS_SPEED - output;
	rightSpeed = constrain(rightSpeed, CONSTRAIN_LOW, CONSTRAIN_HIGH);

	leftSpeed = LEFT_SS_SPEED + output;
	leftSpeed = constrain(leftSpeed, CONSTRAIN_LOW, CONSTRAIN_HIGH);

	if (runMotor)	//Start the motor
	{
		digitalWrite(LDIR2, LOW);
		digitalWrite(LDIR1, HIGH);
		digitalWrite(RDIR2, LOW);
		digitalWrite(RDIR1, HIGH);
		analogWrite(RPWM, rightSpeed);
		analogWrite(LPWM, leftSpeed);
	}
	else	//Stop the motor
	{
		digitalWrite(LDIR1, LOW);
		digitalWrite(LDIR2, LOW);
		analogWrite(LPWM, 0);
		digitalWrite(RDIR1, LOW);
		digitalWrite(RDIR2, LOW);
		analogWrite(LPWM, 0);
	}

	delay(100);
}

//Webpages
void handleRoot() {
	setDirection = server.arg("direction").toFloat();
	setDistance = server.arg("distance").toInt();

	if (DEBUG_SERIAL)
	{
		Serial.println("$$$$$$$$$$$$$$$$$$$");
		Serial.print(setDirection);
		Serial.print(" ");
		Serial.println(setDistance);
		Serial.println("$$$$$$$$$$$$$$$$$$$");
	}

	server.send(200, "text/html", webpage);
}
