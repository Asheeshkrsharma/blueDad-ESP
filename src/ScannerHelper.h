/*
 * BLEScan.cpp
 *
 *  Created on: Jul 1, 2017
 *      Author: kolban
 */
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include "BLEAdvertisedDevice.h"
#include "BLEUtils.h"
#include "GeneralUtils.h"
#include <esp_err.h>
#include <map>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#endif

// Stores the advertised txpower and the known mac address to filter the
// scanning.
// the txpower can be unknown initially, if it is not in a valid range (-26,
// -100),
// then the scanning routing also parses the advertised data for that.
std::map<std::string, float> m_knowns = {};
// This holds the advertisedDevice. Note this is a global variable
// because we want to minimize the memory allocation and reallocation
// for this.
BLEAdvertisedDevice *newAdvertisedDevice = new BLEAdvertisedDevice();
/**
 * Constructor
 */
BLEScan::BLEScan()
{
  m_scan_params.scan_type = BLE_SCAN_TYPE_PASSIVE; // Default is a passive scan.
  m_scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  m_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  m_pAdvertisedDeviceCallbacks = nullptr;
  m_stopped = true;
  m_wantDuplicates = false;
  setInterval(100);
  setWindow(100);
} // BLEScan

int getTxPower(uint8_t *payload, size_t total_len)
{
  uint8_t length;
  uint8_t ad_type;
  uint8_t sizeConsumed = 0;
  bool finished = false;
  bool hasTxPower = false;
  int txPower;
  while (!finished)
  {
    length = *payload;          // Retrieve the length of the record.
    payload++;                  // Skip to type
    sizeConsumed += 1 + length; // increase the size consumed.

    if (length != 0)
    { // A length of 0 indicates that we have reached the end.
      ad_type = *payload;
      payload++;
      length--;
      if ((ad_type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) & (length == 25))
      {
        auto d = std::string(reinterpret_cast<char *>(payload), length);
        char *pHex =
            BLEUtils::buildHexData(nullptr, (uint8_t *)d.data(), length);
        sscanf((pHex + strlen(pHex) - 2), "%X", &txPower);
        free(pHex);
        if (txPower > 0x80)
        {
          txPower = txPower - 0x100;
        }
        else
        {
          txPower = -txPower;
        }
        hasTxPower = true;
        finished = true;
        break;
      }
      payload += length;
    } // Length <> 0
    if ((sizeConsumed >= total_len) | hasTxPower)
      finished = true;
  } // !finished
  if (hasTxPower)
  {
    return txPower;
  }
  else
  {
    return -1;
  }
} // parseAdvertisement

/**
 * @brief Handle GAP events related to scans.
 * @param [in] event The event type for this event.
 * @param [in] param Parameter data for this event.
 */
void BLEScan::handleGAPEvent(esp_gap_ble_cb_event_t event,
                             esp_ble_gap_cb_param_t *param)
{
  if (event == ESP_GAP_BLE_SCAN_RESULT_EVT)
  {
    // Event that indicates that the duration allowed for the search has
    // completed or that we have been asked to stop.
    if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
    {
      m_stopped = true;
      m_semaphoreScanEnd.give();
      if (m_scanCompleteCB != nullptr)
      {
        m_scanCompleteCB(m_scanResults);
      }
      return;
    }
    if ((param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) &
        !m_stopped)
    { // Result that has arrived back from a
      // Scan inquiry.
      BLEAddress advertisedAddress(param->scan_rst.bda);
      auto adStr = advertisedAddress.toString();
      if (m_knowns.find(adStr) != m_knowns.end())
      {
        newAdvertisedDevice->setAddress(advertisedAddress);
        newAdvertisedDevice->setRSSI(param->scan_rst.rssi);

        // Check if the txpower known for device is valid or not.
        if (m_knowns[adStr] == 0.0)
        {
          // Parse the manufacturing data
          int txPower = getTxPower((uint8_t *)param->scan_rst.ble_adv,
                                   param->scan_rst.adv_data_len +
                                       param->scan_rst.scan_rsp_len);
          if (txPower != -1)
          {
            m_knowns[adStr] = txPower;
            newAdvertisedDevice->setTXPower(txPower);
          }
        }
        else
        {
          newAdvertisedDevice->setTXPower(m_knowns[adStr]);
        }
        m_pAdvertisedDeviceCallbacks->onResult(*newAdvertisedDevice);
      }
    }
  }
  return;
} // gapEventHandler

/**
 * @brief Should we perform an active or passive scan?
 * The default is a passive scan.  An active scan means that we will wish a scan
 * response.
 * @param [in] active If true, we perform an active scan otherwise a passive
 * scan.
 * @return N/A.
 */
void BLEScan::setActiveScan(bool active)
{
  if (active)
  {
    m_scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
  }
  else
  {
    m_scan_params.scan_type = BLE_SCAN_TYPE_PASSIVE;
  }
} // setActiveScan

/**
 * @brief Set the call backs to be invoked.
 * @param [in] pAdvertisedDeviceCallbacks Call backs to be invoked.
 * @param [in] wantDuplicates  True if we wish to be called back with
 * duplicates.  Default is false.
 */
void BLEScan::setAdvertisedDeviceCallbacks(
    BLEAdvertisedDeviceCallbacks *pAdvertisedDeviceCallbacks, bool wantDuplicates)
{
  m_wantDuplicates = wantDuplicates;
  m_pAdvertisedDeviceCallbacks = pAdvertisedDeviceCallbacks;
} // setAdvertisedDeviceCallbacks

/**
 * @brief Set the interval to scan.
 * @param [in] The interval in msecs.
 */
void BLEScan::setInterval(uint16_t intervalMSecs)
{
  m_scan_params.scan_interval = intervalMSecs / 0.625;
} // setInterval

/**
 * @brief Set the window to actively scan.
 * @param [in] windowMSecs How long to actively scan.
 */
void BLEScan::setWindow(uint16_t windowMSecs)
{
  m_scan_params.scan_window = windowMSecs / 0.625;
} // setWindow

/**
 * @brief Start scanning.
 * @param [in] duration The duration in seconds for which to scan.
 * @param [in] scanCompleteCB A function to be called when scanning has
 * completed.
 * @param [in] are we continue scan (true) or we want to clear stored devices
 * (false)
 * @return True if scan started or false if there was an error.
 */
bool BLEScan::start(uint32_t duration, void (*scanCompleteCB)(BLEScanResults),
                    bool is_continue)
{
  m_semaphoreScanEnd.take(std::string("start"));
  m_scanCompleteCB =
      scanCompleteCB; // Save the callback to be invoked when the scan completes.

  //  if we are connecting to devices that are advertising even after being
  //  connected, multiconnecting peripherals then we should not clear map or we
  //  will connect the same device few times
  if (!is_continue)
  {
    for (auto _dev : m_scanResults.m_vectorAdvertisedDevices)
    {
      delete _dev.second;
    }
    m_scanResults.m_vectorAdvertisedDevices.clear();
  }
  esp_err_t errRc = ::esp_ble_gap_set_scan_params(&m_scan_params);

  if (errRc != ESP_OK)
  {
    m_semaphoreScanEnd.give();
    return false;
  }

  errRc = ::esp_ble_gap_start_scanning(duration);

  if (errRc != ESP_OK)
  {
    m_semaphoreScanEnd.give();
    return false;
  }

  m_stopped = false;

  return true;
} // start

/**
 * @brief Start scanning and block until scanning has been completed.
 * @param [in] duration The duration in seconds for which to scan.
 * @return The BLEScanResults.
 */
BLEScanResults
BLEScan::start(uint32_t duration, bool is_continue)
{
  if (start(duration, nullptr, is_continue))
  {
    m_semaphoreScanEnd.wait("start"); // Wait for the semaphore to release.
  }
  return m_scanResults;
} // start

/**
 * @brief Stop an in progress scan.
 * @return N/A.
 */
void BLEScan::stop()
{
  esp_err_t errRc = ::esp_ble_gap_stop_scanning();

  m_stopped = true;
  m_semaphoreScanEnd.give();

  if (errRc != ESP_OK)
  {
    return;
  }

} // stop

// delete peer device from cache after disconnecting, it is required in case we
// are connecting to devices with not public address
void BLEScan::erase(BLEAddress address)
{
  BLEAdvertisedDevice *advertisedDevice =
      m_scanResults.m_vectorAdvertisedDevices.find(address.toString())->second;
  m_scanResults.m_vectorAdvertisedDevices.erase(address.toString());
  delete advertisedDevice;
}

/**
 * @brief Return the count of devices found in the last scan.
 * @return The number of devices found in the last scan.
 */
int BLEScanResults::getCount()
{
  return m_vectorAdvertisedDevices.size();
} // getCount

/**
 * @brief Return the specified device at the given index.
 * The index should be between 0 and getCount()-1.
 * @param [in] i The index of the device.
 * @return The device at the specified index.
 */
BLEAdvertisedDevice
BLEScanResults::getDevice(uint32_t i)
{
  uint32_t x = 0;
  BLEAdvertisedDevice dev = *m_vectorAdvertisedDevices.begin()->second;
  for (auto it = m_vectorAdvertisedDevices.begin();
       it != m_vectorAdvertisedDevices.end(); it++)
  {
    dev = *it->second;
    if (x == i)
      break;
    x++;
  }
  return dev;
}

BLEScanResults
BLEScan::getResults()
{
  return m_scanResults;
}

void BLEScan::clearResults()
{
  for (auto _dev : m_scanResults.m_vectorAdvertisedDevices)
  {
    delete _dev.second;
  }
  m_scanResults.m_vectorAdvertisedDevices.clear();
}

#endif /* CONFIG_BT_ENABLED */