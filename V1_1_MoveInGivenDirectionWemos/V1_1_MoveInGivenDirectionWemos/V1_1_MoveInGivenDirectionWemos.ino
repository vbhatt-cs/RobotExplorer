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
#include <WiFiUdp.h>

//Set to 1 if debugging in serial
const bool DEBUG_SERIAL = 1;

// Pins used on Wemos D1 R2
#define LPWM D8
#define LDIR1 D7
#define LDIR2 D6

#define RPWM D5
#define RDIR1 D4
#define RDIR2 D0

#define ENCODER D3

#define PROXIMITY A0

// Left and Right steady state speed
const int RIGHT_SS_SPEED = 300;
const int LEFT_SS_SPEED = 300;
const int CONSTRAIN_LOW = 250;
const int CONSTRAIN_HIGH = 350;

// Left and Right speed
int rightSpeed = 300;
int leftSpeed = 300;

//PID Values
double curDirection = 0, output;

// PID Multipliers
float kp = 1;
float ki = 0;
float kd = 0.2;

double setDirection = 0;	//Reference direction
int setDistance = 0;		//Reference distance

int curDistance = 0;
bool runMotor = true;
bool rotateMotor = true;
bool wall = false;
bool startFlag = true;

FaBo9Axis fabo_9axis;	// For MPU9250 readings
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method
float deltat = 0.0f;						// integration interval for both filter schemes
uint32_t lastUpdate = 0;	// used to calculate integration interval
uint32_t Now = 0;							// used to calculate integration interval
uint32_t startTime = 0;
float magBias[3] = { 7.49, 3.28, -18.94};
float magScale[3] = { 39.96, 41.53, 33.60};

//Setup PID
PID wheelControl(&curDirection, &output, &setDirection, kp, ki, kd, DIRECT);

//SSID of access point
const char *SSID = "Wemos";
const char *password = "qwer321!";

//UDP stuff
WiFiUDP Udp;
//IPAddress ip(192, 168, 137, 69);
//IPAddress gateway(192, 168, 137, 1);
//IPAddress subnet(255, 255, 255, 0);
unsigned int localUdpPort = 4210;  // local port to listen on

void setup() {
	if (DEBUG_SERIAL)
		Serial.begin(9600);

    pinMode(ENCODER, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER), encISR, FALLING);

	pinMode(RPWM, OUTPUT);
	pinMode(RDIR1, OUTPUT);
	pinMode(RDIR2, OUTPUT);

	pinMode(LPWM, OUTPUT);
	pinMode(LDIR1, OUTPUT);
	pinMode(LDIR2, OUTPUT);

	pinMode(PROXIMITY, INPUT);

	//Start the WiFi access point
	if (DEBUG_SERIAL)
	{
		Serial.println();
		Serial.print("Configuring access point...");
	}

	WiFi.softAP(SSID);
	/*WiFi.begin(SSID, password);
	WiFi.config(ip, gateway, subnet);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		if (DEBUG_SERIAL)
			Serial.print(".");
	}*/

	Udp.begin(localUdpPort);

	if (DEBUG_SERIAL)
		Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), localUdpPort);
    
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
	wheelControl.SetOutputLimits(-1000, 1000);
	wheelControl.SetMode(AUTOMATIC);

	startTime = micros();
}

void loop() {
	handleUDP();	//Handle UDP messages

	//Read MPU9250 and get compass heading
	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	
	curDirection = getYaw();
	if (curDirection < -170)
		curDirection = abs(curDirection);
 
	runMotor = curDistance < setDistance;

	if (rotateMotor)
		wall = false;
	else
		wall = analogRead(PROXIMITY) < 512;

	//Compute the PID output
	wheelControl.Compute();

	if (DEBUG_SERIAL)
	{
		Serial.print(" yaw = "); Serial.println(curDirection);
		Serial.print(" ref = "); Serial.println(setDirection);
		Serial.print(" output = "); Serial.println(output);
		Serial.print(" distance = "); Serial.println(curDistance);
	}

	//Set the PWM values for the motor
	//
	//Output added to one wheel and subtracted from another wheel to rotate the bot
	//Change the sign of output (if needed) to make the bot rotate in a direction which reduces the error. 
	//
	//The motor was not moving for PWM below CONSTRAIN_LOW and moving too fast for PWM over CONSTRAIN_HIGH
	//So constrain the value inside that range
	rightSpeed = RIGHT_SS_SPEED + output;
	rightSpeed = constrain(rightSpeed, CONSTRAIN_LOW, CONSTRAIN_HIGH);

	leftSpeed = LEFT_SS_SPEED - output;
	leftSpeed = constrain(leftSpeed, CONSTRAIN_LOW, CONSTRAIN_HIGH);

	if (runMotor && !wall)	//Start the motor
	{
		if (micros() - startTime < 8000000)
		{
			analogWrite(RPWM, 0);
			analogWrite(LPWM, 0);
		}
		else if ((abs(setDirection - curDirection) > 5 && abs(setDirection - curDirection) < 355) && rotateMotor)
		{
			if (setDirection - curDirection > 0 && setDirection - curDirection < 180)
			{
				digitalWrite(RDIR2, LOW);
				digitalWrite(RDIR1, HIGH);
				digitalWrite(LDIR2, LOW);
				digitalWrite(LDIR1, HIGH);
				analogWrite(RPWM, RIGHT_SS_SPEED);
				analogWrite(LPWM, LEFT_SS_SPEED);
			}
			else
			{
				digitalWrite(LDIR1, LOW);
				digitalWrite(LDIR2, HIGH);
				digitalWrite(RDIR1, LOW);
				digitalWrite(RDIR2, HIGH);
				analogWrite(RPWM, RIGHT_SS_SPEED);
				analogWrite(LPWM, LEFT_SS_SPEED);
			}
		}
		else
		{
			rotateMotor = false;
			digitalWrite(LDIR1, LOW);
			digitalWrite(LDIR2, HIGH);
			digitalWrite(RDIR2, LOW);
			digitalWrite(RDIR1, HIGH);
			analogWrite(RPWM, rightSpeed);
			analogWrite(LPWM, leftSpeed);
		}
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

	myDelay(200);
}

void myDelay(int ms) {
    while(ms--) {
        if((!runMotor || wall) && !startFlag)
            return;
        runMotor = curDistance < setDistance;
        if (rotateMotor)
            wall = false;
        else
            wall = analogRead(PROXIMITY) < 512;
        delay(1);
    }
}

//Handle UDP request
void handleUDP() {
	int packetSize = Udp.parsePacket();
	if (packetSize)
	{
		// receive incoming UDP packets
		char incomingPacket[2048];
		int len = Udp.read(incomingPacket, 2048);
		if (len > 0)
		{
			incomingPacket[len] = 0;
		}

		if (DEBUG_SERIAL)
		{
			Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
			Serial.printf("UDP packet contents: %s\n", incomingPacket);
		}

		String req(incomingPacket);

		// send back a reply, to the IP address and port we got the packet from
		// Sensor data requested
		if (req == "Sensors")
		{
			String reply = String(curDirection) + " " + curDistance + " " + wall + " " + (!runMotor);
			Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
			Udp.write(reply.c_str());
			Udp.endPacket();
		}
		//New movement command
		else
		{
			String dir, dis;
			dir = req.substring(0, req.indexOf(' '));
			dis = req.substring(req.indexOf(' ') + 1);

			float setDirOld = setDirection;
			setDirection = dir.toFloat();
			setDistance = dis.toInt();

			if (setDistance > 0)
				curDistance = 0;

            startFlag = false;

			if(setDirection != setDirOld)
				rotateMotor = true;

			if (DEBUG_SERIAL)
			{
				Serial.println("$$$$$$$$$$$$$$$$$$$");
				Serial.print(setDirection);
				Serial.print(" ");
				Serial.println(setDistance);
				Serial.println("$$$$$$$$$$$$$$$$$$$");
			}
		}
	}
}

float getYaw()
{
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;

	fabo_9axis.readAccelXYZ(&ax, &ay, &az);
	fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
	fabo_9axis.readMagnetXYZ(&mx, &my, &mz);

	mx = (mx - magBias[0]) / magScale[0];
	my = (my - magBias[1]) / magScale[1];
	mz = (mz - magBias[2]) / magScale[2];

	/////////
	//Serial.print("mx: ");
	//Serial.print(mx);
	//Serial.print(" my: ");
	//Serial.print(my);
	//Serial.print(" mz: ");
	//Serial.println(mz);

	/*if (DEBUG_SERIAL)
	{
	Serial.print("ax: ");
	Serial.print(ax);
	Serial.print(" ay: ");
	Serial.print(ay);
	Serial.print(" az: ");
	Serial.println(az);
	Serial.print("gx: ");
	Serial.print(gx);
	Serial.print(" gy: ");
	Serial.print(gy);
	Serial.print(" gz: ");
	Serial.println(gz);
	Serial.print("mx: ");
	Serial.print(mx);
	Serial.print(" my: ");
	Serial.print(my);
	Serial.print(" mz: ");
	Serial.println(mz);
	Serial.print("q0 = "); Serial.print(q[0]);
	Serial.print(" qx = "); Serial.print(q[1]);
	Serial.print(" qy = "); Serial.print(q[2]);
	Serial.print(" qz = "); Serial.println(q[3]);
	}*/

	MahonyQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f, my, mx, mz);

	float yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	yaw *= 180.0f / PI;

	return yaw;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	const float Kp = 5.0f;
	const float Ki = 0.1f;

	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

void encISR()
{
	if (!rotateMotor)
		curDistance++;
}
