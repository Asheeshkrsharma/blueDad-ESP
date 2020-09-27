#include <Arduino.h>
#include <BLEDevice.h>

#include "connections.h"
#include "BLE.h"
WIFIMQTTClient wmb;
Scanner scn;

// PUBLISH callback
void sendLocationUpdate(String payload){
  digitalWrite(STATUS_LED, HIGH);
  wmb.sendLocationUpdate(payload);
  Serial.print("...");
  digitalWrite(STATUS_LED, LOW);
}

void setup()
{
  pinMode(STATUS_LED, OUTPUT);
  
  Serial.begin(115200);
  // Connects to a hotspot and estabilshed MQTT
  wmb.connect();

  // Starts a bluetooth scanning routine
  // In the connect routine, we also ask for a list of beacons
  // which is used by the scanner to scope the ble scanning procedure
  scn.setup(wmb.StationaryBeacons);

  // Set a callback which is called when
  // new data is to be published over mqtt.
  scn.setPubCallback(sendLocationUpdate);

  delay(500);
}

// This is a parallel loop.
void loop()
{
  // Needed to maintain the wifi network
  wmb.refresh();
}