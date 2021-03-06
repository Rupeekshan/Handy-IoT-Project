
#include <SoftwareSerial.h>


#define DEBUG true

SoftwareSerial esp8266(0,1);

void setup()
{
  Serial.begin(9600);
  esp8266.begin(9600);

  pinMode(11, OUTPUT);
  digitalWrite(11,LOW);

  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  pinMode(10, OUTPUT);
  digitalWrite(10,LOW);

   sendCommand("AT+RST\r\n",2000,0);
  sendCommand("AT+CWMODE=1\r\n",1000,0);
  sendCommand("AT+CWJAP=\"FaryLink_DE4D3A\",\"\"\r\n",3000,0);
  delay(10000);
  sendCommand("AT+CIFSR\r\n",1000,0);
  sendCommand("AT+CIPMUX=1\r\n",1000,0);
  sendCommand("AT+CIPSERVER=1\r\n",1000,0);

  Serial.println("Server Ready");
}


void loop()
{
 if(esp8266.find("+IPD,"))
 {
  delay(1000);
  int connectionId = esp8266.read()-48;
  esp8266.find("pin=");

  int pinNumber = (esp8266.read()-48)*10;
  pinNumber +=(esp8266.read()-48);

  digitalWrite(pinNumber, !digitalRead(pinNumber));

  String content;
  content = "Pin ";
  content += pinNumber;
  content += " is ";

  if(digitalRead(pinNumber))
  {
    content += "ON";
  }
  else
  {
    content += "OFF";
  }

  sendHTTPResponse(connectionId,content);

  String closeCommand = "AT+CIPCLOSE=";
  closeCommand+=connectionId;
  closeCommand+="\r\n";

  sendCommand(closeCommand,1000,DEBUG);
  }
 } 


String sendData(String command, const int timeout, boolean debug)
{
 String response ="";

  int dataSize = command.length();
  char data[dataSize];
  command.toCharArray(data,dataSize);

  esp8266.write(data,dataSize);
  if(debug)
  {
    Serial.println("\r\n===== HTTP Response From Arduino ======");
    Serial.write("data,dataSize");
    Serial.println("\r\n=======================================");
  }

  long int time = millis();

  while( (time+timeout) > millis())
  {
    while(esp8266.available())
    {
      char c = esp8266.read();
      response+=c;
    }
  }

  if(debug)
  {
    Serial.print(response);
  }

  return response; 

}

void sendHTTPResponse(int connectionId, String content)
{
  String httpResponse;
  String httpHeader;

  httpHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n";
  httpHeader += "Content-length: ";
  httpHeader += content.length();
  httpHeader += "\r\n";
  httpHeader +="Connection: close\r\n\r\n";
  httpResponse = httpHeader + content + " ";
  sendCIPData(connectionId,httpResponse);
}



void sendCIPData(int connectionId, String data)
{
  String cipSend = "AT+CIPSEND";
  cipSend += connectionId;
  cipSend += ",";
  cipSend +=data.length();
  cipSend +="\r\n";
  sendCommand(cipSend,1000,DEBUG);
  sendData(data,1000,DEBUG);
}


String sendCommand(String command, const int timeout, boolean debug)
{
  String response = "";
  esp8266.print(command);
  long int t = millis();

  while( (t+timeout) > millis())
  {
    while(esp8266.available())
    {
      char c = esp8266.read();
      response+=c;
    }
  }

  if(debug)
  {
    Serial.print(response);
  }

  return response;
}


  
