#include <Arduino.h>
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include "MPU6050.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "LittleFS.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h> 
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>

const char *ssid = "xxxx";
const char *password = "xxxx";

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80

#define SEALEVELPRESSURE_HPA (1013.25)
SSD1306Wire display(0x3c, D7, D6);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h
MPU6050 accelgyro;
Adafruit_BMP3XX bmp; // I2C

unsigned long previousMillis=0;
unsigned long prevOneSec=0;
unsigned int interval=10; // Program Scan Rate

int16_t ax, ay, az;
int16_t gx, gy, gz;

float previousExecTime = 0;

File logFile;
int FileCounter = 1;
FSInfo fs_info;
bool WRITE_FLAG = false;

void dumpLogs()
{
  String results = "";
  logFile.close();
  logFile = LittleFS.open("flightData.csv", "r");
  results = "Hello! Flight data in the log is provided below.\n\nUse the Millis column to filter flight and system restart attempts.\n\nFile Size: " + String(logFile.size()) + " Bytes\n\n-------------------------\n\n";
  for (uint i=0;i<logFile.size();i++)
  {
    server.sendContent(String(char(logFile.read())));
  }
  logFile.close();
  logFile = LittleFS.open("flightData.csv", "a");
}

void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}
void handleCalibration()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(az);
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(gz);
  server.send(200, "text/plain", "Gyro calibrated.");
}

void startWrite()
{
  WRITE_FLAG = true;
  logFile.println("Millis,pitch,roll,Temperature,Pressure,Calculated Altitude,Execution Time");
  server.send(200, "text/plain", "Logging Started.");
}

void stopWrite()
{
  WRITE_FLAG = false;
  server.send(200, "text/plain", "Logging Stopped.");
}

void eraseFile()
{
  logFile.close();
  LittleFS.remove("flightData.csv");
  server.send(200, "text/plain", "File erased.");
  delay(500);
  logFile = LittleFS.open("flightData.csv", "a");
}

void setup() {
  WiFi.softAP(ssid, password);
  server.on("/", dumpLogs);
  server.on("", dumpLogs);
  server.on("/purge", eraseFile);
  server.on("/start", startWrite);
  server.on("/stop", stopWrite);
  server.on("/calibrate", handleCalibration);
  server.begin();
  server.onNotFound(handleNotFound);
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleAccelRange(MPU6050_GYRO_FS_2000);
  // Initialising the UI will init the display too.
  display.init();
  Wire.begin(D7, D6);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  accelgyro.setSleepEnabled(false);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.begin();
  LittleFS.begin();
  logFile = LittleFS.open("flightData.csv", "a");
  pinMode(D3,INPUT);
  pinMode(D1,OUTPUT);
  digitalWrite(D1,FALSE);
}

int counter = 1;
void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();
  unsigned long currentMillis2 = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval)
  {
    // clear the display
    // draw the current demo method
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    bmp.performReading();
    bmp.readAltitude(SEALEVELPRESSURE_HPA);
    display.clear();
    float roll = atan2(ay , az) * 180.0 / PI;
    float pitch = atan2(-ax , sqrt(ay * ay + az * az)) * 180.0 / PI;
    display.drawString(0,0,"Roll: " + String(roll) + "°");
    display.drawString(0,10,"Pitch: " + String(pitch) + "°");
    display.drawString(0, 20, "Temp: " + String(bmp.temperature) + " degC");
    display.drawString(0, 30, "Press: " + String(bmp.pressure / 1000.0) + " HPa");
    LittleFS.info(fs_info);
    display.drawString(0,50,"Free: " + String((fs_info.totalBytes - fs_info.usedBytes)/1024) + " kB");
    //display.drawString(0,50, "Alt: " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + " meters");
    unsigned long execTime = millis() - previousExecTime;
    if (WRITE_FLAG)
      logFile.println(String(millis())+","+String(pitch)+","+String(roll)+"," + \
      String(bmp.temperature)+","+String(bmp.pressure) +"," + \
      String(bmp.readAltitude(SEALEVELPRESSURE_HPA))+","+String(execTime));
    display.drawString(0,40, "Scan Time: " + String(execTime) + " ms");
    display.display();
    previousMillis = currentMillis;
    previousExecTime = millis();
  }
  if ((unsigned long)(currentMillis2 - prevOneSec) >= 1000)
  {
    logFile.flush();
    prevOneSec = currentMillis2;
  }
  unsigned long currentMilsForPulse = millis();
  if((currentMilsForPulse - (int(currentMilsForPulse / 1000) * 1000) < 100) && (currentMilsForPulse - (int(currentMilsForPulse / 1000) * 1000) >= 0) && WRITE_FLAG)
  {
    digitalWrite(D1,TRUE);
  }
  else
  {
    digitalWrite(D1,FALSE);
  }
  
}
