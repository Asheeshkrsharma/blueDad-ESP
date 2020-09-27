#include <PubSubClient.h>
#include <WiFi.h>
#include "credentials.h"
#include <ESPRandom.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

class WIFIMQTTClient
{
private:
  WiFiClient wifiClient;
  PubSubClient mqttClient;
  String mqttClientId;
  TaskHandle_t mqttTask;

public:
  String StationaryBeacons;
  // Constructor
  WIFIMQTTClient()
  {
    this->mqttClient.setClient(this->wifiClient);
    uint8_t uuid[16];
    ESPRandom::uuid(uuid);
    this->mqttClientId = ESPRandom::uuidToString(uuid);
  }

  void connect()
  {
    WiFi.disconnect();
    delay(10); // Disconnect if previously connected.

    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
    Serial.print("\nConnected to ");
    Serial.print(STA_SSID);
    Serial.print(" ");
    Serial.print(WiFi.localIP());

    // Now get the list of stationary beacons to be managed.
    Serial.print("\nRequesting server for the list of stationary beacons\n");
    HttpClient client = HttpClient(this->wifiClient, STATIONARY_BEACON_API_URL, STATIONARY_BEACON_API_PORT);

    if (CALLIBRATION_MODE)
    {
      Serial.print("Callibration mode is on:\n");
      client.get((std::string(STATIONARY_BEACON_API_CALLIBRATION_ENDPOINT) + std::string(FACILITY_NAME) + "&lat=" + lat + "&lon=" + lon).c_str());
      int statusCode = client.responseStatusCode();
      if (statusCode == 200)
      {
        this->StationaryBeacons = client.responseBody();
      }
      else
      {
        Serial.print("\nBad server response. Trying again in 5 seconds\n");
        delay(5000);
        this->connect();
      }
    }
    else
    {
      Serial.print("Broadcast mode is on:\n");
      client.get((std::string(STATIONARY_BEACON_API_ENDPOINT) + std::string(FACILITY_NAME)).c_str());
      int statusCode = client.responseStatusCode();
      if (statusCode == 200)
      {
        this->StationaryBeacons = client.responseBody();
      }
      else
      {
        Serial.print("\nBad server response. Trying again in 5 seconds\n");
        delay(5000);
        this->connect();
      }
    }
    // connect to MQTT server
    this->mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    this->startMQTT();
    delay(1000);
  }

  boolean connected()
  {
    return (this->mqttClient.connected()) | (WiFi.status() == WL_CONNECTED);
  }

  void startMQTT()
  {
    Serial.print("\nWaiting for MQTT connection");
    // Loop until we're reconnected
    while (!this->mqttClient.connected())
    {
      Serial.print(".");
      // Attempt to connect, just a name to identify the client
      if (this->mqttClient.connect(this->mqttClientId.c_str(), MQTT_USER, MQTT_PASS))
      {
        Serial.print("\nConnected as ");
        Serial.print(this->mqttClientId);
        Serial.print("@");
        Serial.print(MQTT_SERVER);
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(this->mqttClient.state());
        Serial.println(" trying again in 5 seconds");
        delay(5000);
      }
    }
  }

  void sendLocationUpdate(String foundDevices)
  {
    uint8_t message_char_buffer[MQTT_MAX_PACKET_SIZE];
    foundDevices.getBytes(message_char_buffer, foundDevices.length() + 1);
    this->mqttClient.beginPublish(LOCATION_UPDATE_TOPIC, foundDevices.length(), false);
    this->mqttClient.print(foundDevices.c_str());
    this->mqttClient.endPublish();
  }

  void refresh()
  {
    if (!this->connected())
    {
      this->startMQTT();
      delay(1000);
    }
    this->mqttClient.loop();
  }
};