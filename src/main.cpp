#include "ArduinoJson.h"
#include "esp_adc_cal.h"
#include "secrets.h"
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define DHTPIN 18
#define DHTTYPE DHT22
#define MQTT_PORT 1883
#define BAT_ADC 2

// Conversion factor for milliseconds to seconds
#define MS_TO_S_FACTOR 1000
#define TIME_TO_SLEEP_MS 60000
#define MIN_TIME_TO_SLEEP_MS 50000

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *mqttServer = MQTT_SERVER;
const char *sensorLocation = SENSOR_LOCATION;

const char *ntpServer = "time.google.com";
const long gmtOffset_sec = 3600 * 1;
const int daylightOffset_sec = 3600 * 1;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int skipCount = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100,
                           &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void mqttConnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    log_d("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "esp32client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      log_d("connected");
    } else {
      log_e("Could not connect to MQTT server, rc=%d", client.state());
      delay(500);
    }
  }
}

void readSensor(JsonDocument &payload) {
  // Initialize sensor
  dht.begin();

  // Get temperature event and print its value.
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    log_e("Error reading temperature!");
  } else {
    payload["temp"] = event.temperature;
    log_i("temperature: %.1f°C", event.temperature);
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    log_e("Error reading humidity!");
  } else {
    payload["hum"] = event.relative_humidity;
    log_i("humidity: %.1f%%", event.relative_humidity);
  }
}

void setup() {
  // Increment boot number and print it every reboot
  ++bootCount;
  log_i("Boot number: %d", bootCount);

  uint16_t outerRetry = 0;
  while (WiFi.status() != WL_CONNECTED && outerRetry++ < 4) {
    uint16_t innerRetry = 0;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED && innerRetry++ < 4) {
      log_d("WiFi status retry: %u", innerRetry);
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      break;
    } else {
      log_d("WiFi begin retry: %u", outerRetry);
      WiFi.disconnect();
      delay(500);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    log_e("WiFi connect failed. Skipping sensor update.");
    skipCount++;
  } else {
    log_i("WiFi connected: %s", WiFi.localIP().toString().c_str());

    // Connect to MQTT server
    client.setServer(mqttServer, MQTT_PORT);

    DynamicJsonDocument payload(1024);
    payload["sen"] = "DHT22";
    readSensor(payload);

    uint32_t battery = readADC_Cal(analogRead(BAT_ADC)) * 2;
    payload["battery"] = battery;

    String json;
    serializeJson(payload, json);

    if (!client.connected()) {
      mqttConnect();
    }
    auto topic = String("/home/sensors/") + sensorLocation;
    if (!client.publish(topic.c_str(), json.c_str(), true)) {
      log_e("MQTT publish failed");
    } else {
      log_d("MQTT message: %s", json.c_str());
    }
    client.disconnect();
    delay(250);
  }

  WiFi.disconnect();

  const long timeToSleep =
      std::max(TIME_TO_SLEEP_MS - (long)millis(), (long)MIN_TIME_TO_SLEEP_MS);
  log_d("Entering deep sleep for %ldms", timeToSleep);

  esp_sleep_enable_timer_wakeup(timeToSleep * MS_TO_S_FACTOR);
  esp_deep_sleep_start();

  log_e("This should never be printed");
}

void loop() {
  // Nothing to do here
}
