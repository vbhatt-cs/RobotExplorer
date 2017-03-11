#include "ESP8266.h"

ESP8266::ESP8266(byte rx, byte tx, bool debug = 1) :DEBUG_SERIAL(debug), esp8266(rx, tx)
{
	webpage = "<html><form method=\"get\">\n";
	webpage += "Direction (angle): <input type=\"number\" name=\"direction\" value=0 min=-180 max=180 step=0.1><br>\n";
	webpage += "Distance (encoder count): <input type=\"number\" name=\"distance\" value=0 min=0><br><br>\n";
	webpage += "<input type=\"submit\" value=\"Submit\">\n";
	webpage += "</form></html>";
}

ESP8266::ESP8266(byte rx, byte tx, String file, bool debug = 1) :
	DEBUG_SERIAL(debug), esp8266(rx, tx), webpage(file) {}

void ESP8266::init(int baud = 9600)
{
	esp8266.begin(baud); // your esp's baud rate might be different

	sendData("AT+RST\r\n", 2000); // reset module
	sendData("AT+CWMODE=2\r\n", 1000); // configure as access point
	sendData("AT+CIFSR\r\n", 1000); // get ip address
	sendData("AT+CIPMUX=1\r\n", 1000); // configure for multiple connections
	sendData("AT+CIPSERVER=1,80\r\n", 1000); // turn on server on port 80
}

void ESP8266::getData(double &direction, int &distance)
{
	if (esp8266.available()) // check if the esp is sending a message 
	{
		if (esp8266.find("+IPD,"))
		{
			delay(1000);

			int connectionId = esp8266.read() - 48; // subtract 48 because the read() function returns 
													// the ASCII decimal value and 0 (the first decimal number) starts at 48

			String cipSend = "AT+CIPSEND=";
			cipSend += connectionId;
			cipSend += ",";
			cipSend += webpage.length();
			cipSend += "\r\n";

			String webRequest = sendData(cipSend, 1000);
			sendData(webpage, 1000);
			String searchString = "direction=";
			direction = webRequest.substring(
				webRequest.indexOf(searchString) + searchString.length(), webRequest.indexOf('&')).toDouble();
			searchString = "distance=";
			distance = webRequest.substring(
				webRequest.indexOf(searchString) + searchString.length(), webRequest.indexOf("HTTP") - 1).toInt();

			if (DEBUG_SERIAL)
			{
				Serial.println("Direction Distance:");
				Serial.print(direction);
				Serial.print(" ");
				Serial.println(distance);
			}

			String closeCommand = "AT+CIPCLOSE=";
			closeCommand += connectionId; // append connection id
			closeCommand += "\r\n";

			sendData(closeCommand, 3000);
		}
	}
}

String ESP8266::sendData(String command, const int timeout)
{
	String response = "";

	esp8266.print(command); // send the read character to the esp8266

	long int time = millis();
	while ((time + timeout) > millis())
	{
		while (esp8266.available())
		{

			// The esp has data so display its output to the serial window 
			char c = esp8266.read(); // read the next character.
			response += c;
		}
	}

	if (DEBUG_SERIAL)
	{
		Serial.println("-----------------------------");
		Serial.println(response);
		Serial.println("-----------------------------");
	}

	return response;
}
