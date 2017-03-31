// ESP8266.h
/*
Library for communication between ESP8266 and Arduino using software serial
Code based on http://allaboutee.com/2014/12/30/esp8266-and-arduino-webserver/
*/

#ifndef _ESP8266_h
#define _ESP8266_h

#include <SoftwareSerial.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class ESP8266 {
public:
	//Constructor. rx is the RX of arduino. tx is the TX of arduino.
	//RX = 10 and TX = 11 for Uno
	ESP8266(byte rx, byte tx, bool debug = 1);
	ESP8266(byte rx, byte tx, String file = "webpage.html", bool debug = 1);

	void init(int baud = 9600);
	void getData(double &direction, int &distance);

private:
	String webpage;
	SoftwareSerial esp8266;
	const bool DEBUG_SERIAL;

	String sendData(String command, const int timeout);
};


#endif

