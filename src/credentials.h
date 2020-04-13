#ifndef __CREDENTIALS_H__
#define __CREDENTIALS_H__

// Access point parameters
// #define STA_SSID "AndroidAP5056"
// #define STA_PASS "tjne9022"
#define STA_SSID "Muller Crew"
#define STA_PASS "butterflies"
#define FACILITY_NAME "flat8"

// MQTT Settings
#define MQTT_SERVER "maqiatto.com"
#define MQTT_PORT 1883
#define MQTT_USER "proof1234@gmail.com"
#define MQTT_PASS "123321qwe"
#define LOCATION_UPDATE_TOPIC "proof1234@gmail.com/assets"


// #define MQTT_SERVER "192.168.86.109"
// #define MQTT_PORT 1883
// #define MQTT_USER "ankitseal"
// #define MQTT_PASS "pablo123"
// #define LOCATION_UPDATE_TOPIC "proof1234@gmail.com/assets"



// BLE SCANNER OPTIONS
#define BLE_SCAN_TIME_SECONDS 5 // How much time do we spend scanning once
#define BLE_SCAN_EXPIRATION_TIME_SECONDS 2.5 // The beacons with data older than this will be ignored.
#define BLE_MAX_CONSEQUTIVE_READINGS 6 // How many consecutive readings to we take per beacon.

// Used to get a list of beacon mac addresses
// only those beacons will be scanned for
// much of the parsing is ignored. Thus improves
// efficiency.
#define STATIONARY_BEACON_API_URL "192.168.1.108"
#define STATIONARY_BEACON_API_PORT 3000
#define STATIONARY_BEACON_API_ENDPOINT "/api/v1/beacons?q="

// Callibration mode
// Under this mode, we place the beacon at specific coordinates
// these readings are then used to collect callibration data
// for curvefitting/model training for RSSI-Distancing.
#define CALLIBRATION_MODE 0
#define lat "-2.579034003156417"
#define lon "51.477120261718369"
// Status LED
#define STATUS_LED 22 // Change this to the LED pin used to indicate that the node is scanning.

#undef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 3000
#endif