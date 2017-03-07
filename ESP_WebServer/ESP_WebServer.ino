#include <SoftwareSerial.h>
 
#define DEBUG true
 
SoftwareSerial esp8266(10,11); // make RX Arduino line is pin 10, make TX Arduino line is pin 11.
                             // This means that you need to connect the TX line from the esp to the Arduino's pin 10
                             // and the RX line from the esp to the Arduino's pin 11
void setup()
{
  Serial.begin(9600);
  esp8266.begin(9600); // your esp's baud rate might be different

  sendData("AT+RST\r\n",2000,DEBUG); // reset module
  sendData("AT+CWMODE=2\r\n",1000,DEBUG); // configure as access point
  sendData("AT+CIFSR\r\n",1000,DEBUG); // get ip address
  sendData("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
  sendData("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80
}
 
void loop()
{
  if(esp8266.available()) // check if the esp is sending a message 
  {
    /*
    while(esp8266.available())
    {
      // The esp has data so display its output to the serial window 
      char c = esp8266.read(); // read the next character.
      Serial.write(c);
    } */
    
    if(esp8266.find("+IPD,"))
    {
     delay(1000);
 
     int connectionId = esp8266.read()-48; // subtract 48 because the read() function returns 
                                           // the ASCII decimal value and 0 (the first decimal number) starts at 48
                                           
     String webpage = "<html><form method=\"get\">\n";
     webpage += "Direction (angle): <input type=\"number\" name=\"direction\" value=0 min=-180 max=180 step=0.1><br>\n";
     webpage += "Distance (encoder count): <input type=\"number\" name=\"distance\" value=0 min=0><br><br>\n";
     webpage += "<input type=\"submit\" value=\"Submit\">\n";
     webpage += "</form></html>";
 
     String cipSend = "AT+CIPSEND=";
     cipSend += connectionId;
     cipSend += ",";
     cipSend +=webpage.length();
     cipSend +="\r\n";
     
     String webRequest = sendData(cipSend,1000,DEBUG);
     sendData(webpage,1000,DEBUG);
     float dir = webRequest.substring(webRequest.indexOf("direction=") + 10, webRequest.indexOf('&')).toFloat();
     int dis = webRequest.substring(webRequest.indexOf("distance=") + 9, webRequest.indexOf("HTTP") - 1).toInt();
     Serial.println("$$$$$$$$$$$$$$$$$$$");
     Serial.print(dir);
     Serial.print(" ");
     Serial.println(dis);

     String closeCommand = "AT+CIPCLOSE="; 
     closeCommand+=connectionId; // append connection id
     closeCommand+="\r\n";
     
     sendData(closeCommand,3000,DEBUG);
    }
  }
}
 
 
String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    
    esp8266.print(command); // send the read character to the esp8266
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.println("-----------------------------");
      Serial.println(response);
    }
    
    return response;
}
