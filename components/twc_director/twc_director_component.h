#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"

extern "C" {
#include "twc/twc_frame.h"
#include "twc/twc_core.h"
#include "twc/twc_protocol.h"
}

namespace esphome {
namespace twc_director {

class TWCDirectorComponent;

class TWCDirectorContactorSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent, uint16_t address) {
    this->parent_ = parent;
    this->address_ = address;
  }
 protected:
  void write_state(bool state) override;
  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
};

class TWCDirectorMasterModeSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent) { this->parent_ = parent; }
 protected:
  void write_state(bool state) override;
  TWCDirectorComponent *parent_{nullptr};
};

class TWCDirectorEnableSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent, uint16_t address) {
    this->parent_ = parent;
    this->address_ = address;
  }
 protected:
  void write_state(bool state) override;
  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
};

class TWCDirectorCurrentNumber : public number::Number {
 public:
  enum CurrentType {
    TYPE_MAX,
    TYPE_INITIAL,
    TYPE_SESSION,
    TYPE_GLOBAL_MAX
  };
  TWCDirectorCurrentNumber() = default;
  void set_parent(TWCDirectorComponent *parent, uint16_t address, CurrentType type) {
    this->parent_ = parent;
    this->address_ = address;
    this->type_ = type;
  }
 protected:
  void control(float value) override;
  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
  CurrentType type_{TYPE_INITIAL};
};

class TWCDirectorCurrentButton : public button::Button {
 public:
  enum ButtonType { INCREASE, DECREASE };
  void set_parent(TWCDirectorComponent *parent, uint16_t address, ButtonType type) {
    this->parent_ = parent;
    this->address_ = address;
    this->type_ = type;
  }
 protected:
  void press_action() override;
  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
  ButtonType type_{INCREASE};
};

class TWCDirectorComponent : public Component, public uart::UARTDevice {
 public:
  explicit TWCDirectorComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  static constexpr std::uint16_t MASTER_ID = 0xF00D;

  void set_master_address(std::uint16_t addr);
  std::uint16_t master_address() const { return this->master_address_; }

  void setup() override;
  void loop() override;

  void request_contactor_state(uint16_t address, bool closed);
  void request_current_change(uint16_t address, TWCDirectorCurrentButton::ButtonType type);
  void set_evse_enabled(uint16_t address, bool enabled);

  void add_evse(uint16_t address,
                binary_sensor::BinarySensor *online,
                text_sensor::TextSensor *firmware_version,
                text_sensor::TextSensor *serial_number,
                binary_sensor::BinarySensor *vehicle_connected,
                text_sensor::TextSensor *vehicle_vin,
                sensor::Sensor *meter_current_phase_a,
                sensor::Sensor *meter_current_phase_b,
                sensor::Sensor *meter_current_phase_c,
                sensor::Sensor *meter_voltage_phase_a,
                sensor::Sensor *meter_voltage_phase_b,
                sensor::Sensor *meter_voltage_phase_c,
                sensor::Sensor *meter_energy_total,
                sensor::Sensor *meter_energy_session,
                TWCDirectorContactorSwitch *contactor,
                binary_sensor::BinarySensor *contactor_status,
                sensor::Sensor *available_current_sensor,
                number::Number *initial_current,
                number::Number *max_current,
                sensor::Sensor *max_current_sensor,
                number::Number *session_current,
                sensor::Sensor *session_current_sensor,
                text_sensor::TextSensor *mode,
                text_sensor::TextSensor *status_text,
                text_sensor::TextSensor *status_log,
                TWCDirectorCurrentButton *increase_current_button,
                TWCDirectorCurrentButton *decrease_current_button,
                TWCDirectorEnableSwitch *enable_switch);

  void set_link_ok_sensor(binary_sensor::BinarySensor *sensor) { this->link_ok_sensor_ = sensor; }
  void set_charging_count_sensor(sensor::Sensor *sensor) { this->charging_count_sensor_ = sensor; }

  void set_master_mode_switch(TWCDirectorMasterModeSwitch *sw) {
    this->master_mode_switch_ = sw;
    if (sw != nullptr) sw->set_parent(this);
  }

  bool master_mode_enabled() const {
    return this->master_mode_switch_ != nullptr && this->master_mode_switch_->state;
  }

  std::uint32_t checksum_error_count() const { return this->checksum_error_count_; }

  void set_global_max_current(float max_current_a) { this->global_max_current_a_ = max_current_a; }
  void set_evse_max_current_limit(float max_current_a) { this->evse_max_current_limit_a_ = max_current_a; }

  void set_flow_control_pin(output::BinaryOutput *pin) { this->flow_control_pin_ = pin; }

  void set_global_max_current_control(number::Number *num) {
    this->global_max_current_control_ = num;
    if (num != nullptr) {
      auto *n = static_cast<TWCDirectorCurrentNumber *>(num);
      n->set_parent(this, 0xFFFF, TWCDirectorCurrentNumber::TYPE_GLOBAL_MAX);
    }
  }

  void handle_current_number_control(uint16_t address, float value,
                                      TWCDirectorCurrentNumber::CurrentType type);

  static constexpr std::uint32_t METRICS_UPDATE_INTERVAL_MS = 1000U;
  static constexpr std::size_t MAX_TX_QUEUE_SIZE = 8;
  static constexpr std::uint32_t DECODER_STATS_INTERVAL_MS = 60000U;

  std::uint32_t tx_frames_queued() const { return this->tx_frames_queued_; }
  std::uint32_t tx_frames_dropped() const { return this->tx_frames_dropped_; }
  std::uint32_t tx_encode_failures() const { return this->tx_encode_failures_; }

  struct EvseEntry {
    uint16_t address;
    bool auto_bound{false};
    bool bound{false};

    binary_sensor::BinarySensor *online;
    text_sensor::TextSensor *firmware_version{nullptr};
    text_sensor::TextSensor *serial_number{nullptr};
    binary_sensor::BinarySensor *vehicle_connected{nullptr};
    text_sensor::TextSensor *vehicle_vin{nullptr};
    // NOTE: meter_current_phase_a/b/c are intentionally never set — TWC Gen2
    // FD EB frame has NO per-phase current bytes. The C library getters read
    // the voltage bytes and return wrong values (e.g. 228V misread as 114A).
    sensor::Sensor *meter_current_phase_a{nullptr};  // always nullptr
    sensor::Sensor *meter_current_phase_b{nullptr};  // always nullptr
    sensor::Sensor *meter_current_phase_c{nullptr};  // always nullptr
    sensor::Sensor *meter_voltage_phase_a{nullptr};
    sensor::Sensor *meter_voltage_phase_b{nullptr};
    sensor::Sensor *meter_voltage_phase_c{nullptr};
    sensor::Sensor *meter_energy_total{nullptr};
    sensor::Sensor *meter_energy_session{nullptr};
    TWCDirectorContactorSwitch *contactor{nullptr};
    binary_sensor::BinarySensor *contactor_status{nullptr};
    sensor::Sensor *available_current_sensor{nullptr};
    number::Number *initial_current{nullptr};
    number::Number *max_current{nullptr};
    sensor::Sensor *max_current_sensor{nullptr};
    number::Number *session_current{nullptr};
    sensor::Sensor *session_current_sensor{nullptr};
    text_sensor::TextSensor *mode{nullptr};
    text_sensor::TextSensor *status_text{nullptr};
    text_sensor::TextSensor *status_log{nullptr};
    TWCDirectorCurrentButton *increase_current_button{nullptr};
    TWCDirectorCurrentButton *decrease_current_button{nullptr};
    TWCDirectorEnableSwitch *enable_switch{nullptr};

    bool enabled{true};
    float last_max_current_setpoint_a{-1.0f};
    float last_initial_current_setpoint_a{-1.0f};
    std::uint32_t last_metrics_publish_ms{0};
    int last_status_code{-1};
    bool last_contactor_switch_state{false};
    float last_session_amps{0.0f};
  };

  void publish_binary_sensor_if_changed_(binary_sensor::BinarySensor *sensor, bool value);
  void publish_sensor_if_changed_(sensor::Sensor *sensor, float value, float epsilon = 0.01f);
  void publish_text_sensor_if_changed_(text_sensor::TextSensor *sensor, const std::string &value);
  const char *mode_to_string_(twc_mode_t mode) const;
  EvseEntry *find_evse_(uint16_t address);
  void handle_initial_current_change_(EvseEntry *changed_evse, uint16_t address, float new_value);
  bool handle_broadcast_or_noise_(const uint8_t *frame, size_t len, const char *raw_hex);
  bool update_checksum_and_log_(const uint8_t *frame, size_t len);
  void classify_and_autobind_(const uint8_t *frame, size_t len, const char *raw_hex);
  void publish_last_frame_(const char *raw_hex);
  float get_effective_global_max_current_() const;

 protected:
  TWCDirectorMasterModeSwitch *master_mode_switch_{nullptr};
  binary_sensor::BinarySensor *link_ok_sensor_{nullptr};
  sensor::Sensor *charging_count_sensor_{nullptr};
  output::BinaryOutput *flow_control_pin_{nullptr};
  bool master_mode_last_state_{false};
  uint32_t last_valid_frame_ms_{0};
  static constexpr uint32_t LINK_TIMEOUT_MS = 15000;
  std::uint32_t checksum_error_count_{0};
  bool last_checksum_ok_{false};
  std::uint16_t master_address_{MASTER_ID};
  static constexpr size_t MAX_FRAME_LEN = 256;
  std::vector<EvseEntry> evse_entries_;

  struct PendingTx {
    std::size_t len{0};
    uint8_t buf[2 + 2 * MAX_FRAME_LEN]{};
  };

  std::vector<PendingTx> tx_queue_;
  std::uint32_t next_tx_at_ms_{0};
  std::uint32_t last_rx_ms_{0};

  static constexpr std::uint32_t TX_INTERFRAME_GAP_MS = 5;
  static constexpr std::uint32_t BUS_IDLE_GUARD_MS = 10;

  float global_max_current_a_{0.0f};

  // Per-EVSE maximum current limit.
  // Default 16A matches a standard EU single-phase 16A circuit breaker.
  // Override in yaml with global_twc_max_current: 32 for 32A installs.
  float evse_max_current_limit_a_{16.0f};

  number::Number *global_max_current_control_{nullptr};
  std::uint32_t tx_frames_queued_{0};
  std::uint32_t tx_frames_dropped_{0};
  std::uint32_t tx_encode_failures_{0};
  twc_core_t core_{};

  void process_rx_(std::uint32_t now);
  void update_master_mode_(std::uint32_t now);
  void update_evse_metrics_(std::uint32_t now);
  void run_master_tick_(std::uint32_t now);
  void drain_tx_queue_(std::uint32_t now);
  void handle_rx_frame_(const uint8_t *frame, size_t len, uint32_t now);
  void sync_desired_current_(EvseEntry &evse);
  void update_evse_sensors_(EvseEntry &evse, uint32_t now);
  void publish_sensor_(sensor::Sensor *sensor, float value, float epsilon = 0.01f);
  void publish_nonempty_(text_sensor::TextSensor *sensor, const char *value);
  void format_hex_(const uint8_t *data, size_t len, char *out, size_t out_size);

  // Returns actual amps the car is drawing from the FD E0 slave heartbeat
  // reportedActual field. TWC Gen2 has NO per-phase current sensors — the
  // FD EB frame contains only kWh + voltages. Never use
  // twc_device_get_phase_a/b/c_current_a(); those read voltage bytes.
  float compute_session_amps_(const twc_device_t *dev) const;

  void handle_auto_bind_(uint16_t address, twc_mode_t mode);
  void handle_rx_byte_(uint8_t byte);
  twc_frame_decoder_t decoder_{};
  void complete_frame_(const uint8_t *frame, size_t len);
  void handle_tx_frame_(const uint8_t *frame, size_t len);

  friend void tx_callback_shim(const uint8_t *frame, size_t len, void *user_data);
  friend void negotiation_callback_shim(uint16_t address, float applied_initial_current_a,
                                        std::uint8_t session_id, void *user_data);
  friend void autobind_callback_shim(uint16_t address, twc_mode_t mode, void *user_data);
};

}  // namespace twc_director
}  // namespace esphome
