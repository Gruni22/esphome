#pragma once

#ifdef USE_ESP32

#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble_server/ble_service.h"
#include "esphome/core/component.h"

#include <deque>
#include <span>
#include <string>
#include <vector>

namespace esphome {
namespace api {
class APIConnection;
}  // namespace api

namespace ble_server {

// Service and characteristic UUIDs — must match the BLE client (Android/HA companion).
static constexpr const char *const HA_SERVICE_UUID = "a10d4b1c-bf45-4c2a-9c32-4a8f7e3d1234";
static constexpr const char *const HA_TX_UUID = "a10d4b1c-bf45-4c2a-9c32-4a8f7e3d1235";  // ESP→client (Notify)
static constexpr const char *const HA_RX_UUID = "a10d4b1c-bf45-4c2a-9c32-4a8f7e3d1236";  // client→ESP (Write)

// 247-byte MTU minus 3 bytes ATT overhead = 244 bytes per notification payload.
static constexpr size_t BLE_MAX_CHUNK = 244;
static constexpr uint8_t BLE_CHUNK_MORE = 0x00;
static constexpr uint8_t BLE_CHUNK_FINAL = 0x01;

class BleServer : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_device_name(const std::string &v) { this->device_name_ = v; }
  void set_manufacturer(const std::string &v) { this->manufacturer_ = v; }
  void set_model(const std::string &v) { this->model_ = v; }
  void set_firmware_version(const std::string &v) { this->firmware_version_ = v; }

  // API entry points — called from APIConnection handlers.
  void subscribe_api_connection(api::APIConnection *conn);
  void unsubscribe_api_connection(api::APIConnection *conn);
  api::APIConnection *get_api_connection() const { return this->subscriber_; }
  void on_frame_from_api(const uint8_t *data, size_t len);

 protected:
  void setup_service_();
  void on_ble_write_(std::span<const uint8_t> data);
  void on_ble_disconnect_();
  void send_ble_chunked_(const uint8_t *data, size_t len);

  std::string device_name_;
  std::string manufacturer_;
  std::string model_;
  std::string firmware_version_;

  esp32_ble_server::BLEService *service_{nullptr};
  esp32_ble_server::BLECharacteristic *tx_char_{nullptr};
  esp32_ble_server::BLECharacteristic *rx_char_{nullptr};
  bool service_setup_done_{false};

  std::vector<uint8_t> rx_assembly_;
  // Outbound chunks waiting to go over BLE notify. Drained one-per-loop-tick
  // from loop() so we don't blow past the ESP-IDF GATT TX queue when sending
  // large responses (e.g., the ~31 KB ANS_DEVICES reply: 129 chunks).
  std::deque<std::vector<uint8_t>> tx_queue_;
  api::APIConnection *subscriber_{nullptr};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern BleServer *global_ble_server_component;

}  // namespace ble_server
}  // namespace esphome

#endif  // USE_ESP32
