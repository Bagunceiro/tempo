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

const struct WiFidesc
{
  const char *ssid;
  const char *psk;
} wifinets[] = {{"asgard", "enaLkraP"}, {"asgard_2g", "enaLkraP"}, {NULL, NULL}};

const String controllername = "weather";

const String version = "Weather 0.0.0";
const String compDate = __DATE__;
const String compTime = __TIME__;

WiFiClient wifiClient;
ESP8266WiFiMulti wifimulti;

PubSubClient mqttClient(wifiClient);

const char* mqttBroker = "azrael.local";
const unsigned long sleepTime = 15 * 60 * 1e6; // 15 mins

Adafruit_BME280 bme; // I2C

bool connectToWiFi()
{
  bool connected = true;
  Serial.println("Connect To WiFi");
  for (int net = 0; wifinets[net].ssid != NULL; net++)
  {
    // Serial.printf("Add AP %s/%s\n", ssid.c_str(), psk.c_str());
    wifimulti.addAP(wifinets[net].ssid, wifinets[net].psk);
  }
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
  if (connected)
    Serial.printf("Connected to %s (%s)\n", WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str());
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

  String clientID = controllername + String(millis() % 1000);

  mqttClient.setServer(mqttBroker, 1883);
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

double setPrecision(double value, const unsigned int precision)
{
  int factor = pow(10, precision);
  return (round(value * factor) / factor);
}

typedef StaticJsonDocument<512> weatherdata;

weatherdata &getData(bool bmeOK, bool rtcOK, weatherdata &doc)
{
  DateTime now = rtc.now();
  doc["bme"] = bmeOK;
  doc["rtc"] = rtcOK;
  if (rtcOK)
  {
    doc["timestamp"] = now.timestamp();
  }
  if (bmeOK)
  {
    doc["temperature"] = setPrecision(bme.readTemperature(), 2);
    doc["pressure"] = setPrecision(bme.readPressure() / 100, 2);
    doc["humidity"] = setPrecision(bme.readHumidity(), 2);
  }
  return doc;
}

void sendData(weatherdata doc)
{
  String message;
  serializeJson(doc, message);
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

  StaticJsonDocument<512> doc;
  bool bmestatus = bme.begin(0x76, &Wire);
  bool rtcstatus = rtc.begin();

  getData(bmestatus, rtcstatus, doc);

  if (connectToWiFi())
  {
    if (!rtc.isrunning())
    {
      Serial.println("RTC is not running");
      rtcstatus = false;
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

    sendData(doc);
  }
  else
  {
    Serial.println("Giving up");
  }

  mqttClient.loop();
  mqttClient.flush();

  Serial.println("Going back to sleep");
  mqttClient.disconnect();
  ESP.deepSleep(sleepTime);
}

void loop()
{
}
