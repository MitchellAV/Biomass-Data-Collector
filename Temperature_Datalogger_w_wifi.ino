#include "SoftwareSerial.h"
String ssid = "SJSU_Guest";
String password = ""; 
#define rxPin 9
#define txPin 8
SoftwareSerial esp(rxPin,txPin);// RX, TX
String data;
String server = "biogasproject.000webhostapp.com"; // www.example.com
String uri = "/addData.php";// our example is /esppost.php
int wifiCheck = 0;
int maxCheck = 10;



String id = "0";
String timedata = "0";
String ambTemp = "0";
String tankTemp= "0";
String adTemp = "0";
String tankErr= "0";
String adErr= "0";
String scfm= "0";
String cfm= "0";
String massFlow= "0";
String irradiance="0" ;
String ph="0" ;
String rpm= "0";
//"id=0&timedata=5/8-16:51:0&ambTemp=10.15&tankTemp=20.12&adTemp=0.00&cfm=0.00&massFlow=0.00&irradiance=0.00&ph=-2.62&rpm=0&tankErr=0&adErr=0";

bool posted = false;
//SD
#include <SPI.h>
//RTC
#include <Wire.h>
#include "RTClib.h"
RTC_PCF8523 rtc;
const int chipSelect = 10;
const String fileName = "data.csv";
//Thermocouple
#include "Adafruit_MAX31855.h"
float Ctemp = 0;
float Htemp = 0;
// digital IO pins.
#define MAXCLK 7
#define MAXDO 6
#define MAXCSH 3
#define MAXCSC 4
// initialize the Thermocouple
Adafruit_MAX31855 thermocoupleH(MAXCLK, MAXCSH, MAXDO);
Adafruit_MAX31855 thermocoupleC(MAXCLK, MAXCSC, MAXDO);
// Mass flow Meter
#define flowPin A0
#define pressurePin A1
const float density = .656;
const float CFMtoSI = 0.000471947;
const float a = -29.804;
const float b = 7.4991;
const float flowOffset = .11;
// Solar Cell
#define solarPin A2
// PH Sensor
#define phPin A3
//unsigned long duration = 1;
boolean prevState = HIGH;
int loopcount = 0;
// Seconds per interval
int interval = 0;
// Testing
const boolean testing = true;
void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  esp.begin(115200);
  Serial.begin(115200);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // correct to current time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // wait for MAX chip to stabilize

  reset();
  connectWifi();
  esp.println("AT+CWMODE=3");
  delay(1000);
  esp.println("AT+CIPMUX=0");
  delay(1000);
}
void loop() {
  float C = 0;
  float H = 0;
  float sumAmb = 0;
  float avgAmb = 0;
  float sumH = 0;
  float sumC = 0;
  float sumCFM = 0;
  float scfm = 0.0;
  float cfm = 0.0;
  float massFlowRate = 0.0;
  float solar = 0;
  float PH = 0;
  float sumFlow = 0;
  float sumSolar = 0;
  float sumPH = 0;
  int countH = 0;
  int countC = 0;
  int CnanCount = 0;
  int HnanCount = 0;
  posted = false;
  String timeInfo = "";
  DateTime now = rtc.now();
  int cSec = now.second();
  timeInfo = String(now.month()) + "-" + String(now.day()) + "-" + String(now.hour()) + ":" + String(now.minute()) + ":" + String(cSec);
  Serial.println(cSec);

//data = String("id=0&timedata=5/8-16:51:0&ambTemp=10.15&tankTemp=20.12&adTemp=0.00&cfm=0.00&massFlow=0.00&irradiance=0.00&ph=-2.62&rpm=0&tankErr=0&adErr=0");

data = "id=0&timedata=0&ambTemp=0&tankTemp=0&adTemp=0&tankErr=0&adErr=0&cfm=0&massFlow=0&irradiance=0&ph=0&rpm=0";
//data = "id="+id+"&timedata="+timedata+"&ambTemp="+ambTemp+"&tankTemp="+tankTemp+"&adTemp="+adTemp+"&tankErr="+tankErr+"&adErr="+adErr+"&scfm="+scfm+"&cfm="+cfm+"&massFlow="+massFlow+"&irradiance="+irradiance+"&ph="+ph+"&rpm="+rpm;/

  while (true) {
   httppost();
  }
  if (String(cSec) == "0") {
    interval = 0;
    int secCheck = 0;
    while (secCheck <= 55) {
      Serial.println(loopcount);
      if (loopcount == 0) {
        posted = true;
      }
      if (!posted) {
        // while(true){
        //   httppost();
        // }
        Serial.println(data);
        httppost();
      }
      // Ambient Temp
      float ambC = thermocoupleC.readInternal();
      float ambH = thermocoupleH.readInternal();
      avgAmb = (ambC + ambH) / 2;
      sumAmb += avgAmb;
      // Hot and cold temps
      C = thermocoupleC.readCelsius();
      H = thermocoupleH.readCelsius();
      // Cold error handling
      if (isnan(C)) {
        CnanCount++;
      }
      else {
        sumC += C;
        countC++;
      }
      // Hot error handling
      if (isnan(H)) {
        HnanCount++;
      }
      else {
        sumH += H;
        countH++;
      }
      // Flow meter
      scfm = massFlowSensor(flowPin);
      cfm = scfm *  ((273.15 + avgAmb) / (273.15 + 20.1833));
      sumCFM += cfm;
      massFlowRate = density * cfm * CFMtoSI * 1000;
      sumFlow += massFlowRate;
      // Solar Cell
      solar = solarCell(solarPin);
      sumSolar += solar;
      // PH Sensor
      PH = phSensor(phPin);
      sumPH += PH;
      now = rtc.now();
      secCheck = now.second();
      interval++;

      Serial.println(secCheck);
    }
    avgAmb = avgSensor(sumAmb, interval);
    H = avgSensor(sumH, countH);
    C = avgSensor(sumC, countC);
    cfm= avgSensor(sumCFM, interval);
    massFlowRate= avgSensor(sumFlow, interval);
    solar= avgSensor(sumSolar, interval);
    PH= avgSensor(sumPH, interval);
    data =String("id=0&timedata="+String(timeInfo)+"&ambTemp="+String(avgAmb)+"&tankTemp="+String(C)+"&adTemp="+String(H)+"&cfm="+String(cfm)+"&massFlow="+String(massFlowRate)+"&irradiance="+String(solar)+"&ph="+String(PH)+"&rpm=0"+"&tankErr="+String(CnanCount)+"&adErr="+String(HnanCount));
    Serial.println(data);
    loopcount++;
  }

}
float massFlowSensor(int flowPin) {
  float flowValue = analogRead(flowPin);
  float flowVoltage = (float)flowValue * (5.0 / 1023.0);
  float flowCurrent = (flowVoltage * pow(10, 3)) / (250.0);
  flowCurrent -= flowOffset;
  float scfm = posFilter(a + b * flowCurrent);
  return scfm;
}
float solarCell(int solarPin) {
  float abit = analogRead(solarPin);
  float tempVolt = abit * (.0048875);
  float rad = tempVolt * (2263.46);
  return rad;
}
float phSensor(int phPin) {
  unsigned long phRead = analogRead(phPin);
  float ph = 0.0342 * phRead - 6.383;
  return ph;
}
float posFilter(float value) {
  if (value < 0) {
    value = 0.0;
  }
  return value;
}
float avgSensor(float sensorSum, float interval) {
  float avg = sensorSum / interval;
  return avg;
}
//reset the esp8266 module
void reset() {
  esp.println("AT+RST");
  delay(1000);
  if (esp.find("OK") ) Serial.println("Module Reset");
}
//connect to your wifi network
void connectWifi() {
  String cmd = "AT+CWJAP=\"" + ssid + "\",\"" + password + "\"";
  esp.println(cmd);
  Serial.println(esp.readString());
  delay(2000);
  if (esp.find("OK")) {
    Serial.println("Connected!");
  }
  else {
    Serial.println("Cannot connect to wifi");
    connectWifi();
  }
}
void httppost () {
  esp.println("AT+CIPSTART=\"TCP\",\"" + server + "\",80");//start a TCP connection.
  Serial.println("AT+CIPSTART=\"TCP\",\"" + server + "\",80");
  delay(100);
  if (esp.find("OK")) {
    wifiCheck = 0;
    Serial.println("TCP connection ready");
    String postRequest ="POST " + String(uri) + " HTTP/1.1\r\n" +
      "Host: " + String(server) + "\r\n" +
      "Accept: *" + "/" + "*\r\n" +
      "Content-Length: " + String(data.length()) + "\r\n" +
      "Content-Type: application/x-www-form-urlencoded\r\n" +
      "\r\n";
      //+ String(data));
    String sendCmd = "AT+CIPSEND=";//determine the number of caracters to be sent.
    esp.print(sendCmd);
    esp.println(postRequest.length());
    if (esp.find(">")) {
      Serial.println("Sending..");
      esp.print(postRequest);
      Serial.println(postRequest);
      if (esp.find("SEND OK")) {
        Serial.println("Packet sent");
        while (esp.available()) {
          String tmpResp = esp.readString();
          Serial.println(tmpResp);
        }
      }
    }
  }
  else{
    Serial.println("Connection not Established");
    if (wifiCheck > maxCheck) {
      connectWifi();
      wifiCheck = 0;
    }
    wifiCheck++;
  }
  // close the connection
  esp.println("AT+CIPCLOSE");
  Serial.println(wifiCheck);
}
