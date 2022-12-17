#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <Adafruit_Sensor.h>
#include <dht.h>
#include "esp_adc_cal.h"

// SERIAL settings
#define SERIAL_SPEED 115200

// WIFI settings
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

// Mosquitto MQTT Broker settings
#define MQTT_HOST IPAddress(192, 123, 123, 123)
#define MQTT_PORT 1883
#define MQTT_ID "MQTT_ID"
#define MQTT_USERNAME "MQTT_USERNAME"
#define MQTT_PASSWORD "MQTT_PASSWORD"
#define MQTT_TOPIC "MQTT_TOPIC"

#define SLEEP_TIME 3*60*1000000ULL

#define DHTPIN D4
#define DHTTYPE DHT22

#define DELIMITER ";"

WiFiClient net;
MQTTClient client;
DHT dht(DHTPIN, DHTTYPE);

int analogBattery = A0;
int g_vref = 1100;

struct SensorData {
  float temperature;
  float relative_humidity;
  float absolute_humidity;
  float dew_point_temperature;
  float battery_voltage;
};

SensorData sd = {0.0, 0.0, 0.0, 0.0, 0.0}; 


// https://github.com/espressif/arduino-esp32/issues/1675
// change in WiFiSTA.cpp
// conf.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;     
// to
// conf.sta.scan_method = WIFI_FAST_SCAN;
// reduce the connection time from 3 sec to 1.5 sec       

void Log(String logMessage)
{
  Serial.println(String(millis()) + " : " + logMessage);
}

bool ConnectToWlan(const char* ssid, const char* passphrase)
{
    unsigned long startTime = millis();

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid, passphrase);
    
    Log("MAC: " + WiFi.macAddress());

    for (int i=0; i<200; i++)
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        unsigned long endTime = millis();
        Log("WiFi connected in " + String(endTime - startTime) + " milliseconds");
        Log("IP-Addr: " + WiFi.localIP().toString());
        return true;
      }
      delay(10);
    } 

    unsigned long endTime = millis();
    Log("WiFi not connected, timeout after " + String(endTime - startTime) + " milliseconds");
    return false;
}

bool DisconnectFromWlan()
{
  Log("WiFi disconnected");
  return WiFi.disconnect(true, true);
}

void BeetleStart()
{
  Serial.begin(SERIAL_SPEED);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   
  Log("Beetle start");
}

void BeetleEnd()
{
  digitalWrite(LED_BUILTIN, LOW);  
  Log("Beetle end");
}

bool SendMqttData(String mqttData)
{
  unsigned long startTime = millis();
  client.begin(MQTT_HOST , MQTT_PORT, net);
  for (int i=0; i<200; i++)
  {
    if (client.connect(MQTT_ID, MQTT_USERNAME, MQTT_PASSWORD))
      {
        Log("MQTT connected");
        bool result = client.publish(MQTT_TOPIC, mqttData, true, 1);
        if (result)
        {
          Log("MQTT send");
        }
        else{
          Log("MQTT not send");
          Log(String(client.lastError()));
        }
        return(result);
      }
    delay(10);
  }
  unsigned long endTime = millis();
  Log("MQTT not connected, timeout after " + String(endTime - startTime) + " milliseconds");
  return false;
}

// https://www.wetterochs.de/wetter/feuchte.html
// https://www.heise.de/ratgeber/Bastelprojekt-Taupunkt-Lueftungssystem-bauen-und-Keller-trockenlegen-6356018.html?seite=2

bool ReadSensorData()
{

  dht.begin();

  sensors_event_t event;

  sd.temperature = dht.readTemperature();
  sd.relative_humidity = dht.readHumidity();

  float a = 0.0;
  float b = 0.0;

  if (sd.temperature >= 0.0)
  {
    a = 7.5;
    b = 237.3; 
  }
  else {
    a = 7.6;
    b = 240.7;
  }

  // Sättigungsdampfdruck in hPa
  float SDD = 6.1078 * pow10((a*sd.temperature)/(b+sd.temperature));

  // Dampfdruck in hPa
  float DD = sd.relative_humidity/100 * SDD;

  // Taupunkttemperatur in °C
  float v = log10(DD/6.1078);
  sd.dew_point_temperature = (b*v) / (a-v);

  // Universelle Gaskonstante J/(kmol*K) 
  float R = 8314.3;

  // Molekulargewicht des Wasserdampfes
  float mw = 18.016;

  // Temperatur in Kelvin
  float TK = sd.temperature + 273.15;

  // absolute Feuchte in g Wasserdampf pro m3 Luft
  sd.absolute_humidity =  1000 * mw/R * (100*DD)/TK;

  Log("Temperature: " + String(sd.temperature )+ " °C");
  Log("Relative Humidity: " + String(sd.relative_humidity) + " %%");
  Log("Absolute Humidity: " + String(sd.absolute_humidity) + " g/m3");
  Log("Dew Point Temperature: " + String(sd.dew_point_temperature) + " °C");

  return true;
}



float readBattery() {
  uint32_t value = 0;
  int rounds = 11;
  esp_adc_cal_characteristics_t adc_chars;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  switch(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars)) {
    case ESP_ADC_CAL_VAL_EFUSE_TP:
      Serial.println("Characterized using Two Point Value");
      break;
    case ESP_ADC_CAL_VAL_EFUSE_VREF:
      Serial.printf("Characterized using eFuse Vref (%d mV)\r\n", adc_chars.vref);
      break;
    default:
      Serial.printf("Characterized using Default Vref (%d mV)\r\n", 1100);
  }

  //to avoid noise, sample the pin several times and average the result
  for(int i=1; i<=rounds; i++) {
    value += adc1_get_raw(ADC1_CHANNEL_0);
  }
  value /= (uint32_t)rounds;

  //due to the voltage divider (1M+1M) values must be multiplied by 2
  //and convert mV to V
  return esp_adc_cal_raw_to_voltage(value, &adc_chars)*2.0/1000.0;
}



void setup()
{
  BeetleStart();

  sd.battery_voltage = readBattery();
  Log("Battery voltage: " + String(sd.battery_voltage) + " V");

  if (ReadSensorData())
  {
    if (ConnectToWlan(WIFI_SSID, WIFI_PASSWORD))
    {
      String mqttData = "";
      mqttData += String(sd.temperature);
      mqttData += DELIMITER;
      mqttData += String(sd.relative_humidity);
      mqttData += DELIMITER;
      mqttData += String(sd.absolute_humidity);
      mqttData += DELIMITER;
      mqttData += String(sd.dew_point_temperature);
      mqttData += DELIMITER;
      mqttData += String(sd.battery_voltage);
    
      SendMqttData(mqttData);
    }
    DisconnectFromWlan();
  }
  BeetleEnd();

  esp_sleep_enable_timer_wakeup(SLEEP_TIME);
  esp_deep_sleep_start();

}

void loop()
{
}