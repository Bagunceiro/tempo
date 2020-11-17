#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

const String ssid = "asgard_2g";
const String psk = "enaLkraP";
const String controllername = "weather";

const String version = "Weather 0.0.0";
const String compDate = __DATE__;
const String compTime = __TIME__;

WiFiClient wifiClient;
ESP8266WiFiMulti wifimulti;

PubSubClient mqttClient(wifiClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

bool connectToWiFi()
{
  bool connected = true;
  Serial.println("Connect To WiFi");
  Serial.printf("Connect to %s/%s\n", ssid.c_str(), psk.c_str());
  wifimulti.addAP(ssid.c_str(), psk.c_str());
  wifimulti.run();
  unsigned int now = millis();
  while (wifimulti.run() != WL_CONNECTED)
  {
    if ((millis() - now) > 10000)
    {
      Serial.println("Connection to WiFi failed");
      connected = false;
      break;
    }
    delay(100);
  }
  Serial.print("Connected ");
  Serial.println(connected);
  return connected;
}

void messageReceived(char *fullTopic, byte *payload, unsigned int length)
{
  String msg;
  for (unsigned int i = 0; i < length; i++)
  {
    msg += (char)(payload[i]);
  }
  Serial.printf("MQTT Messsage\n");
}

void initMQTT()
{
  Serial.print("Connecting to MQTT ");

  String clientID = String("ctlr_") + String(millis() % 1000);

  mqttClient.setServer("192.168.0.108", 1883);
  mqttClient.setCallback(messageReceived);

  if (mqttClient.connect(clientID.c_str(), "ctlr", "fatty"))
  {
    Serial.println("MQTT connected");
  }
  else
  {
    Serial.print("Failed: ");
    Serial.println(mqttClient.state());
  }
}

String setPrecision(double value, const unsigned int precision)
{
  int factor = pow(10, precision);
   long v2 = round(value * factor);
   String f = String(v2/factor);
   if (factor > 1) 
   {
     f += ".";
     String fraction = String(v2 % factor);
     while (fraction.length() < precision)
     {
       fraction = String("0") + fraction;
     }
     f += fraction;
   }
   // Serial.printf("%f approx %s\n", value, f.c_str());
   return f;
}

void sendData()
{
  // Serial.printf("Sending %s\n", temp.c_str());
  //   mqttClient.publish("/home/temp", temp.c_str(), true);

  DateTime now = rtc.now();
  StaticJsonDocument<512> doc;
  doc["timestamp"] = now.timestamp();
  doc["temperature"] = setPrecision(bme.readTemperature(), 2);
  doc["pressure"] = setPrecision(bme.readPressure()/100, 2);
  doc["humidity"] = setPrecision(bme.readHumidity(), 2);
  Serial.println("");
  String message;
  serializeJson(doc, message);
  Serial.println(message);
  mqttClient.publish("home/weather", message.c_str(), message.length());
}

void i2cscan()
{
  for (byte address = 0; address <= 127; address++)
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (address == 0)
    {
      Serial.print("\n    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    }
    if (address % 16 == 0)
    {
      Serial.println("");
      Serial.print(address, HEX);
      Serial.print(":");
    }
    Serial.print(" ");
    if (error == 2)
      Serial.print("  ");
    else
    {
      if (error < 0x10)
        Serial.print("0");
      Serial.print(error, HEX);
    }
  }
  Serial.println();
}

void setup()
{
  WiFi.mode(WIFI_STA);
  Serial.begin(9600);
  Serial.println("");
  Wire.begin(2, 0);
  // i2cscan();

  Serial.println(version);
  if (connectToWiFi())
  {
    // Serial.println("Do your stuff");
    unsigned status;

    // default settings
    if (!rtc.begin())
    {
      Serial.println("Couldn't find RTC");
    }
    if (!rtc.isrunning())
    {
      Serial.println("RTC is NOT running!");
    }
    else
    {
      DateTime now = rtc.now();
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" (");
      Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
      Serial.print(") ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();
    }

    initMQTT();

    status = bme.begin(0x76, &Wire);
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status)
    {
      Serial.println("Could not find a valid BME280 sensor");
      Serial.print("SensorID was: 0x");
      Serial.println(bme.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }
    else
    {
      Serial.print("Temperature = ");
      Serial.print(bme.readTemperature());
      Serial.println(" *C");

      Serial.print("Pressure = ");

      Serial.print(bme.readPressure() / 100.0F);
      Serial.println(" hPa");

      Serial.print("Approx. Altitude = ");
      Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");

      Serial.print("Humidity = ");
      Serial.print(bme.readHumidity());
      Serial.println(" %");

      Serial.println();
      sendData();
    }
  } 
  else
  {
    Serial.println("Giving up");
  }

  mqttClient.loop();
  mqttClient.flush();


  Serial.println("Going back to sleep");
  mqttClient.disconnect();
  ESP.deepSleep(20e6);
}

void loop()
{
}
