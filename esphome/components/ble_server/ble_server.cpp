#include "ble_server.h"

#ifdef USE_ESP32

#include "esphome/components/api/api_connection.h"
#include "esphome/components/api/api_pb2.h"
#include "esphome/components/esp32_ble/ble.h"
#include "esphome/components/esp32_ble_server/ble_2902.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ble_server {

static const char *const TAG = "ble_server";

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BleServer *global_ble_server_component = nullptr;

float BleServer::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH; }

void BleServer::setup() {
  global_ble_server_component = this;

  if (!this->device_name_.empty() && esp32_ble::global_ble != nullptr) {
    esp32_ble::global_ble->set_name(this->device_name_.c_str());
  }

  esp32_ble_server::global_ble_server->on_disconnect([this](uint16_t conn_id) { this->on_ble_disconnect_(); });
}

void BleServer::loop() {
  if (!esp32_ble_server::global_ble_server->is_running())
    return;
  if (!this->service_setup_done_) {
    this->setup_service_();
    this->service_setup_done_ = true;
  }

  // Drain at most one BLE notification per loop tick. Notifications have no
  // wire-level flow control — firing 129 chunks back-to-back overruns the
  // ESP-IDF GATT TX queue and chunks past depth ~7 get dropped silently. By
  // throttling to one per tick (~16 ms) we get ~2 s for a 31 KB response,
  // well within the app's per-command timeout.
  if (this->tx_queue_.empty() || this->tx_char_ == nullptr ||
      esp32_ble_server::global_ble_server->get_connected_client_count() == 0) {
    return;
  }
  this->tx_char_->set_value(std::move(this->tx_queue_.front()));
  this->tx_char_->notify();
  this->tx_queue_.pop_front();
}

void BleServer::setup_service_() {
  ESP_LOGD(TAG, "Creating BLE service %s", HA_SERVICE_UUID);
  this->service_ =
      esp32_ble_server::global_ble_server->create_service(esp32_ble::ESPBTUUID::from_raw(HA_SERVICE_UUID), true);

  this->tx_char_ = this->service_->create_characteristic(HA_TX_UUID, esp32_ble_server::BLECharacteristic::PROPERTY_READ |
                                                                        esp32_ble_server::BLECharacteristic::PROPERTY_NOTIFY);
  this->tx_char_->add_descriptor(new esp32_ble_server::BLE2902());

  this->rx_char_ =
      this->service_->create_characteristic(HA_RX_UUID, esp32_ble_server::BLECharacteristic::PROPERTY_WRITE |
                                                            esp32_ble_server::BLECharacteristic::PROPERTY_WRITE_NR);
  this->rx_char_->on_write([this](std::span<const uint8_t> data, uint16_t /*conn_id*/) { this->on_ble_write_(data); });

  // Enqueue (don't call start() directly): the BLE server's loop drives
  // start() repeatedly until every characteristic is created, then transitions
  // the service to RUNNING and adds its UUID to advertising. Calling start()
  // once would only kick off the first characteristic and stall — leaving the
  // service UUID out of the advertising packet.
  esp32_ble_server::global_ble_server->enqueue_start_service(this->service_);
}

void BleServer::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Server:");
  ESP_LOGCONFIG(TAG, "  Device Name: %s", this->device_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Manufacturer: %s", this->manufacturer_.c_str());
  ESP_LOGCONFIG(TAG, "  Model: %s", this->model_.c_str());
  ESP_LOGCONFIG(TAG, "  Firmware Version: %s", this->firmware_version_.c_str());
  ESP_LOGCONFIG(TAG, "  Service UUID: %s", HA_SERVICE_UUID);
}

void BleServer::subscribe_api_connection(api::APIConnection *conn) {
  if (this->subscriber_ != nullptr && this->subscriber_ != conn) {
    ESP_LOGW(TAG, "Replacing existing API subscriber");
  }
  this->subscriber_ = conn;
}

void BleServer::unsubscribe_api_connection(api::APIConnection *conn) {
  if (this->subscriber_ == conn) {
    this->subscriber_ = nullptr;
  }
}

void BleServer::on_frame_from_api(const uint8_t *data, size_t len) {
  if (this->tx_char_ == nullptr) {
    ESP_LOGW(TAG, "Frame from API dropped: BLE service not yet running");
    return;
  }
  this->send_ble_chunked_(data, len);
}

void BleServer::send_ble_chunked_(const uint8_t *data, size_t len) {
  // Build all chunks and push them onto tx_queue_. The actual notify() calls
  // happen one-per-loop-tick from loop() to respect the ESP-IDF GATT TX queue.
  static constexpr size_t PAYLOAD_PER_CHUNK = BLE_MAX_CHUNK - 1;
  size_t offset = 0;
  while (offset < len) {
    size_t remaining = len - offset;
    size_t this_chunk = std::min(remaining, PAYLOAD_PER_CHUNK);
    bool is_final = (offset + this_chunk) == len;
    std::vector<uint8_t> chunk;
    chunk.reserve(this_chunk + 1);
    chunk.push_back(is_final ? BLE_CHUNK_FINAL : BLE_CHUNK_MORE);
    chunk.insert(chunk.end(), data + offset, data + offset + this_chunk);
    this->tx_queue_.push_back(std::move(chunk));
    offset += this_chunk;
  }
  // Empty payload: still queue a single FINAL chunk so the client sees a zero-length frame.
  if (len == 0) {
    this->tx_queue_.push_back(std::vector<uint8_t>{BLE_CHUNK_FINAL});
  }
}

void BleServer::on_ble_write_(std::span<const uint8_t> data) {
  if (data.empty())
    return;
  uint8_t header = data[0];
  this->rx_assembly_.insert(this->rx_assembly_.end(), data.begin() + 1, data.end());
  if (header != BLE_CHUNK_FINAL)
    return;

  if (this->subscriber_ == nullptr) {
    ESP_LOGD(TAG, "BLE frame dropped: no API subscriber");
    this->rx_assembly_.clear();
    return;
  }

  api::BleServerFrameResponse resp;
  resp.set_data(this->rx_assembly_.data(), this->rx_assembly_.size());
  this->subscriber_->send_message(resp);
  this->rx_assembly_.clear();
}

void BleServer::on_ble_disconnect_() {
  this->rx_assembly_.clear();
  // Drop any pending outbound chunks — the next client will send their own
  // requests and shouldn't receive bytes addressed to the previous one.
  this->tx_queue_.clear();
}

}  // namespace ble_server
}  // namespace esphome

#endif  // USE_ESP32
