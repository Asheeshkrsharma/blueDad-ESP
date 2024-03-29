#include <ArduinoJson.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include "kalman.h"
#include <algorithm>
#include "ScannerHelper.h"
struct deviceData
{
    std::vector<double> rssi;   // Vector to store the rssi
    double txPower;             // TxPower of the device
    unsigned long lastUpdateTS; // Keep track of the timestamp at which the device was found.
};
class Scanner : public BLEAdvertisedDeviceCallbacks
{
private:
    BLEScan *pBLEScan;
    std::map<std::string, deviceData> mapRSSI;
    TaskHandle_t scannerTask;
    std::string macAddress;
    uint8_t message_char_buffer[MQTT_MAX_PACKET_SIZE];
    Kalman_Filter_Distance *kalman;
    void (*publish)(String payload);
    void scanOnce()
    {
        Serial.print("\nScanner task running on core ");
        Serial.print(xPortGetCoreID());
        Serial.print(" as ");
        Serial.print(this->macAddress.c_str());
        Serial.print("\n");
        for (;;)
        {
            BLEScanResults foundDevices = this->pBLEScan->start(BLE_SCAN_TIME_SECONDS, false);
            this->pBLEScan->stop();
            int numBeacons = 0;
            // Number of beacons which were updated recently.
            for (auto device : this->mapRSSI)
                numBeacons += ((
                                  (millis() - device.second.lastUpdateTS) * 0.001 < BLE_SCAN_TIME_SECONDS))
                                  ? (1)
                                  : (0);

            // Parse new data
            if (numBeacons >= 4)
            {
                this->parseData();
            }
            else
            {
                Serial.println("No new updates");
            }
        }
    }
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        std::string address = advertisedDevice.getAddress().toString();
        int txPower = advertisedDevice.getTXPower();
        // Append the data.
        if (mapRSSI.find(address) == mapRSSI.end())
        {
            auto rssi = std::vector<double>({(double)advertisedDevice.getRSSI()});
            rssi.reserve(BLE_MAX_CONSEQUTIVE_READINGS);
            mapRSSI.insert(std::pair<std::string, deviceData>(
                address,
                {
                    rssi,
                    (float)txPower,
                    millis(),
                }));
        }
        else
        {
            // We should check here, for the current size of the vector.
            // i.e. if it is greater than BLE_MAX_CONSEQUTIVE_READINGS
            // we remove the old ones.
            if (mapRSSI[address].rssi.size() == BLE_MAX_CONSEQUTIVE_READINGS)
            {
                mapRSSI[address].rssi.erase(mapRSSI[address].rssi.begin());
            }
            mapRSSI[address].rssi.push_back((double)advertisedDevice.getRSSI());
            mapRSSI[address].lastUpdateTS = millis();
            mapRSSI[address].txPower = txPower;
        }
    }
    // Task wrapper
    static void startTaskImpl(void *_this)
    {
        static_cast<Scanner *>(_this)->scanOnce();
    }

public:
    String beaconData;
    void setup(String stationaryBeacons)
    {
        BLEDevice::init("");
        this->pBLEScan = BLEDevice::getScan();
        this->pBLEScan->setActiveScan(true);
        this->pBLEScan->setInterval(100);
        this->pBLEScan->setWindow(100);
        this->pBLEScan->setAdvertisedDeviceCallbacks(this);
        this->macAddress = BLEDevice::getAddress().toString();

        // The key value pairs are used to filter certain addresses.
        // The value indicates the txPower of different beacons
        if (CALLIBRATION_MODE)
        {
            DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
            DeserializationError error = deserializeJson(doc, stationaryBeacons);
            m_knowns = {};
            if (!error)
                for (auto beacon : doc.as<JsonArray>())
                {
                    m_knowns.insert(std::pair<std::string, std ::vector<float>>(beacon[0].as<char *>(), {0.0, beacon[1]}));
                }
        }
        else
        {
            DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
            DeserializationError error = deserializeJson(doc, stationaryBeacons);
            m_knowns = {};
            if (!error)
                for (auto beacon : doc.as<JsonArray>())
                {
                    m_knowns.insert(std::pair<std::string, std ::vector<float>>(beacon.as<char *>(), {0.0, 0.0}));
                }
        }

        // else ? Reboot may be?
        xTaskCreatePinnedToCore(
            this->startTaskImpl, /* Task function. */
            "Scanner",           /* name of task. */
            10000,               /* Stack size of task */
            this,                /* parameter of the task */
            1,                   /* priority of the task */
            &this->scannerTask,  /* Task handle to keep track of created task */
            0);                  /* pin task to core 1 */

        this->beaconData = ""; // We dont have any data initially.
        this->kalman = new Kalman_Filter_Distance(0.008);
    }

    // This method is used to calculate the ideal rssi
    // Given that the distance to the beacon is know.
    // This is only used duing the callibration process
    // To smoothen the readings and calculating the
    // measurment noise for the Kalman filter.
    double getIdealRSSI(double txPower, double distance)
    {
        /*
         * RSSI = TxPower - 10 * n * lg(d)
         * n = 2 (in free space)
        */
        return txPower - (10 * 2 * log10(distance));
    }

    void parseData()
    {
        DynamicJsonDocument result(MQTT_MAX_PACKET_SIZE);
        result["address"] = this->macAddress;
        result["facility"] = FACILITY_NAME;
        JsonObject rssiData = result.createNestedObject("rssi");

        if (CALLIBRATION_MODE)
        {
            result["lat"] = lat;
            result["lon"] = lon;
        }

        for (auto &device : this->mapRSSI)
        {
            int numVals = device.second.rssi.size();
            // We need at least three values
            if (numVals >= 4)
            {
                // Calculate the mean
                double average = std::accumulate(device.second.rssi.begin(),
                                                 device.second.rssi.end(), 0.0) /
                                 device.second.rssi.size();

                double variance = 0.0;
                // During callibration, we calculate the measurement noise
                // using its deviation from the ideal rssi at the known
                // distance.
                if (CALLIBRATION_MODE)
                {
                    // During the callibration, we know the distance between
                    // the beacon and this device. So we calculte the variance
                    // in the recived rssi based on what the rssi is supposed
                    // to be in an ideal world.
                    // Then we use the variance to adjust the measurement noise
                    // in the kalman filter.
                    auto idealRSSI = getIdealRSSI(device.second.txPower, m_knowns[device.first][1]);
                    // Calculate the variance
                    std::vector<double> diff(device.second.rssi.size());
                    std::transform(device.second.rssi.begin(),
                                   device.second.rssi.end(), diff.begin(),
                                   [idealRSSI](double x) { return x - idealRSSI; });
                    variance = std::sqrt(
                        std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) /
                        device.second.rssi.size());
                }
                else
                {
                    // Calculate the normal variance
                    std::vector<double> diff(device.second.rssi.size());
                    std::transform(device.second.rssi.begin(),
                                   device.second.rssi.end(), diff.begin(),
                                   [average](double x) { return x - average; });
                    variance = std::sqrt(
                        std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) /
                        device.second.rssi.size());
                }

                // We push the filtered average 6 - numVals times.
                if (numVals < BLE_MAX_CONSEQUTIVE_READINGS)
                {
                    int N_ = BLE_MAX_CONSEQUTIVE_READINGS - numVals;
                    for (int i = 0; i < N_; i++)
                        device.second.rssi.push_back(average);
                }

                // Make a json object
                JsonObject rootObj = rssiData.createNestedObject(device.first);
                JsonArray deviceData = rootObj.createNestedArray("data");

                // Add the filtered rssi data to the json object.
                this->kalman->reset();
                // Measurement Noise: Represents (electronic, random) noise characteristics of the sensor.
                // It is calculated from the sensor accuracy which is represented using "standard deviation"
                // of measured value from true values (sigma) during the calibration.
                // sigma_sq = sigma^2;
                this->kalman->setMeasurementNoise(variance * variance);
                for (auto &rssi : device.second.rssi)
                {
                    rssi = (double)this->kalman->filter(rssi);
                    deviceData.add(rssi);
                }
                rootObj["txP"] = device.second.txPower;
            }
            // Clear the vector if the data is stale now
            auto condition = ((millis() - device.second.lastUpdateTS) * 0.001 > BLE_SCAN_EXPIRATION_TIME_SECONDS);
            if (condition)
            {
                std::vector<double>().swap(device.second.rssi);
            }
        }

        this->beaconData = "";
        serializeJson(result, this->beaconData);
        if (!(this->beaconData.equals("")))
        {
            this->publish(this->beaconData);
        }
        // Clear the json object
        result.clear();
    }

    void setPubCallback(void (*callback)(String payload))
    {
        this->publish = callback;
    }
};