//SD
#include <SPI.h>
#include <SD.h>

//RTC
#include <Wire.h>
#include "RTClib.h"

RTC_PCF8523 rtc;
const int chipSelect = 10;
const String fileName = "data.csv";

//Thermocouple
#include "Adafruit_MAX31855.h"

double Ctemp = 0;
double Htemp = 0;

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

const double density = .656;
const double CFMtoSI = 0.000471947;

const double a = -29.804;
const double b = 7.4991;

int pressureValue = 0;
double pressureVoltage = 0.0;
double pressurekPa = 0.0;



const double flowOffset = .11;
double pressureOffset = -0.14418;

// Solar Cell
#define solarPin A2

// PH Sensor
#define phPin A3


// IR Sensor
const int IR = 5;


int period = 1000;

//unsigned long duration = 1;
boolean prevState = HIGH;


// Seconds per interval
int interval = 0;
double timeFactor = .5;
double delayTime = (1000.0) * timeFactor;

// Testing
const boolean testing = false;

void setup() {
  Serial.begin(9600);
  pinMode(IR, INPUT);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // correct to current time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // wait for MAX chip to stabilize
  delay(1000);

  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Date/Time,Ambient Temp (C),Storage Tank Thermocouple (C),AD Thermocouple (C), CFM, Mass Flow Rate (g/s), Solar Irradiance (W/m^2), PH, RPM,AD Error,Storage Tank Error");
    dataFile.close();
  }
  else {
    while (1) {
      Serial.println("Unable to open file");
    }
  }
}

void loop() {
  double C = 0;
  double H = 0;

  double sumAmb = 0;
  double avgAmb = 0;

  double sumH = 0;
  double sumC = 0;

  double sumSCFM = 0;
  double avgSCFM = 0;

  double sumCFM = 0;
  double avgCFM = 0;

  double scfm = 0.0;
  double cfm = 0.0;
  double massFlowRate = 0.0;

  double solar = 0;
  double PH = 0;
  double RPM = 0;

  double sumFlow = 0;
  double avgFlow = 0;

  double sumSolar = 0;
  double avgSolar = 0;

  double sumPH = 0;
  double avgPH = 0;

  double sumRPM = 0;
  double avgRPM = 0;
  int readings = 0;
  int count = 0;
  unsigned long prevTime = 0;
  unsigned long prevMillis = 0;

  int countH = 0;
  int countC = 0;

  int CnanCount = 0;
  int HnanCount = 0;

  String timeInfo = "";

  DateTime now = rtc.now();

  int cYear = now.year();
  int cMonth = now.month();
  int cDay = now.day();

  int cHour = now.hour();
  int cMin = now.minute();
  int cSec = now.second();

  timeInfo = String(cMonth) + "/" + String(cDay) + " - " + String(cHour) + ":" + String(cMin) + ":" + String(cSec);
  Serial.println(timeInfo);
  if (String(cSec) == "0") {
    interval = 0;
    int secCheck = 0;
    while (secCheck <= 50) {

      // Ambient Temp
      double ambC = thermocoupleC.readInternal();
      double ambH = thermocoupleH.readInternal();
      avgAmb = (ambC + ambH) / 2;
      sumAmb += ambC;// change before final

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
      sumSCFM += scfm;

      cfm = scfm * (101.3529 / (101.3529 + pressurekPa)) * ((273.15 + avgAmb) / (273.15 + 20.1833));
      sumCFM += cfm;

      massFlowRate = density * cfm * CFMtoSI * 1000;
      sumFlow += massFlowRate;

      // Pressure
      //  pressureValue = analogRead(pressurePin);
      //  pressureVoltage = (double)pressureValue * (5.0 / 1023.0);
      //  pressurekPa = posFilter(5 * ((pressureVoltage / 5.0) - .5) + pressureOffset);

      // Solar Cell
      solar = solarCell(solarPin);
      sumSolar += solar;

      // PH Sensor
      PH = phSensor(phPin);
      sumPH += PH;


      // IR Sensor
      RPM = irSensor(IR, prevState, prevMillis, prevTime, count, period, readings);
      sumRPM += RPM;
//      Serial.println();
//      Serial.print("count = ");
//      Serial.println(count);
//      Serial.print("period: ");
//      Serial.println(period);
//      Serial.print("RPM: ");
//      Serial.println(RPM);
//      Serial.println();


      if (testing) {
        Serial.print("Count: ");
        Serial.println(interval);
        Serial.print("Average Internal Temp = ");
        Serial.println(avgAmb);

        Serial.print("Tank = ");
        Serial.println(C);
        Serial.print("AD = ");
        Serial.println(H);

        Serial.print("CFM: ");
        Serial.println(cfm);
        Serial.print("Mass flow rate g/s: ");
        Serial.println(massFlowRate);
        Serial.print("Solar Irradiance: ");
        Serial.println(solar);
        Serial.print("PH: ");
        Serial.println(PH);
        Serial.print("RPM: ");
        Serial.println(RPM);
        Serial.print("AD error count: ");
        Serial.println(HnanCount);
        Serial.print("Tank error count: ");
        Serial.println(CnanCount);

        Serial.print("Error AD = ");
        Serial.println(thermocoupleC.readError());
        Serial.print("Error Tank = ");
        Serial.println(thermocoupleH.readError());
      }
      now = rtc.now();
      secCheck = now.second();
      count++;
      interval++;
    }

    avgAmb = avgSensor(sumAmb, interval);
    H = avgSensor(sumH, countH);
    C = avgSensor(sumC, countC);
    avgCFM = avgSensor(sumCFM, interval);
    avgFlow = avgSensor(sumFlow, interval);
    avgSolar = avgSensor(sumSolar, interval);
    avgPH = avgSensor(sumPH, interval);
    avgRPM = avgSensor(sumRPM, count);

    // Serial Print
    if (testing) {
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println(timeInfo);
      Serial.print("Interval time: ");
      Serial.println(interval);
      Serial.print("Average Internal Temp = ");
      Serial.println(avgAmb);

      Serial.print("Tank = ");
      Serial.println(C);
      Serial.print("AD = ");
      Serial.println(H);
      Serial.print("CFM: ");
      Serial.println(avgCFM);
      Serial.print("Mass flow rate g/s: ");
      Serial.println(avgFlow);
      Serial.print("Solar Irradiance W/m^2: ");
      Serial.println(avgSolar);
      Serial.print("PH: ");
      Serial.println(avgPH);
      Serial.print("RPM: ");
      Serial.println(avgRPM);
      Serial.print("AD error count: ");
      Serial.println(HnanCount);
      Serial.print("Tank error count: ");
      Serial.println(CnanCount);
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
    }


    // Save to SD Card
    File dataFile = SD.open(fileName, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(timeInfo);
      dataFile.print(",");
      dataFile.print(avgAmb);
      dataFile.print(",");
      dataFile.print(C);
      dataFile.print(",");
      dataFile.print(H);

      dataFile.print(",");
      dataFile.print(avgCFM);
      dataFile.print(",");
      dataFile.print(avgFlow);
      dataFile.print(",");
      dataFile.print(avgSolar);
      dataFile.print(",");
      dataFile.print(avgPH);
      dataFile.print(",");
      dataFile.print(avgRPM);
      if (HnanCount > 0) {
        dataFile.print(",");
        dataFile.print(String(HnanCount));
      } else {
        dataFile.print(",");
        dataFile.print("0");
      }
      if (CnanCount > 0) {
        dataFile.print(",");
        dataFile.print(String(CnanCount));
      } else {
        dataFile.print(",");
        dataFile.print("0");
      }
      dataFile.println();
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening " + fileName);
    }
  }
}


double massFlowSensor(int flowPin) {
  double flowValue = analogRead(flowPin);
  double flowVoltage = (double)flowValue * (5.0 / 1023.0);
  double flowCurrent = (flowVoltage * pow(10, 3)) / (250.0);
  flowCurrent -= flowOffset;
  double scfm = posFilter(a + b * flowCurrent);
  return scfm;
}

double solarCell(int solarPin) {
  float abit = analogRead(solarPin);
  float tempVolt = abit * (.0048875);
  float rad = tempVolt * (2263.46);
  return rad;
}

double phSensor(int phPin) {
  unsigned long phRead = analogRead(phPin);
  float ph = 0.0342 * phRead - 6.383;
  return ph;
}


double irSensor(int IR, boolean prevState, unsigned long prevMillis, unsigned long prevTime, int count, int period, int readings) {
  boolean currentState = digitalRead(IR);
  //  Serial.print("currentState = ");
  //  Serial.println(currentState);
  //  Serial.print("prevState = ");
  //  Serial.println(prevState);
  unsigned long duration;
  if (prevState != currentState) {
    if (currentState == LOW) {
      duration = (millis() - prevMillis);
      //      Serial.print("duration = ");
      //      Serial.println(duration);
      prevMillis = millis();
      //      Serial.print("prevMillis = ");
      //      Serial.println(prevMillis);
      count++;
    }
  }
  prevState = currentState;
  unsigned long currentTime = millis();
//  Serial.print("currentTime = ");
//  Serial.println(currentTime);
//  Serial.print("prevTime = ");
//  Serial.println(prevTime);
//  Serial.print("check = ");
//  Serial.println((unsigned long)(currentTime - prevTime));

  prevTime = currentTime;
  count = 0;
  readings++;
//  Serial.print("readings = ");
//  Serial.println(readings);
  double rpm = (60000000 / (double)duration);
  return rpm;
}

double posFilter(double value) {
  if (value < 0) {
    value = 0.0;
  }
  return value;
}

double avgSensor(double sensorSum, double interval) {
  double avg = sensorSum / interval;
  return avg;
}

double accMap(double x, double x0, double x1, double y0, double y1) {
  return ((y1 - y0) / (x1 - x0)) * (x - x0) + y0;
}
