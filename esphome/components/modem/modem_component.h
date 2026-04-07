#pragma once
#ifdef USE_ESP32
#ifdef USE_ESP_IDF

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/lock_free_queue.h"
#include "esphome/components/network/ip_address.h"

using esphome::esp_log_printf_;  // NOLINT

#include "esp_netif.h"
#include "cxx_include/esp_modem_api.hpp"

namespace esphome {
namespace modem {

enum ModemType {
  MODEM_TYPE_UNKNOWN = 0,
  MODEM_TYPE_SIM7600,
  MODEM_TYPE_SIM800,
};

enum class ModemComponentState {
  TURNING_ON_POWER,
  TURNING_OFF_POWER,
  PWRKEY_CLICK,
  RESET_CLICK,
  SYNC,
  REGISTRATION_IN_NETWORK,
  CONNECTING,
  CONNECTED,
};

enum class ModemEvent : uint8_t {
  TICK = 0,
  TIMEOUT,
  SYNC_OK,
  ATTACH_OK,
  CMUX_OK,
  CMUX_FAIL,
  PPP_GOT_IP,
  PPP_LOST_IP,
  EVENT_COUNT,
};

struct ModemComponentStateTiming {
  uint32_t poll_period;
  uint32_t time_limit;
  constexpr ModemComponentStateTiming(uint32_t poll_period = 0, uint32_t time_limit = 0)
      : poll_period(poll_period), time_limit(time_limit) {}
};

struct ModemQueuedEvent {
  ModemEvent event;
};

class ModemComponent : public Component {
 public:
  ModemComponent();
  void setup() override;
  void loop() override;
  void dump_config() override;
  void dump_connect_params();
  float get_setup_priority() const override;
  bool can_proceed() override;
  bool is_connected();
  bool is_disabled() const;
  bool started{false};
  void set_power_pin(InternalGPIOPin *power_pin);
  void set_pwrkey_pin(InternalGPIOPin *pwrkey_pin);
  void set_type(ModemType type);
  void set_reset_pin(InternalGPIOPin *reset_pin);
  void set_apn(const std::string &apn);
  void set_tx_pin(uint8_t tx_pin);
  void set_rx_pin(uint8_t rx_pin);
  void set_uart_event_task_stack_size(int uart_event_task_stack_size);
  void set_uart_event_task_priority(int uart_event_task_priority);
  void set_uart_event_queue_size(int uart_event_queue_size);
  void set_uart_tx_buffer_size(int uart_tx_buffer_size);
  void set_uart_rx_buffer_size(int uart_rx_buffer_size);
  int get_rssi();
  int get_ber();
  int get_modem_voltage();

  network::IPAddress get_ip_address();
  network::IPAddresses get_ip_addresses();
  const char *get_use_address() const;
  void set_use_address(const std::string &use_address);
  bool powerdown();

 protected:
  static constexpr ModemComponentStateTiming get_state_timing_(ModemComponentState state) {
    switch (state) {
      case ModemComponentState::TURNING_ON_POWER:
      case ModemComponentState::TURNING_OFF_POWER:
        return {2000, 0};
      case ModemComponentState::PWRKEY_CLICK:
      case ModemComponentState::RESET_CLICK:
        return {100, 0};
      case ModemComponentState::SYNC:
      case ModemComponentState::REGISTRATION_IN_NETWORK:
        return {2000, 30000};
      case ModemComponentState::CONNECTING:
        return {2000, 60000};
      case ModemComponentState::CONNECTED:
        return {5000, 0};
    }
    return {};
  }

  static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
  void modem_netif_init_();
  void dte_init_();
  void dce_init_();

  void schedule_timing_event_();
  void process_event_queue_();
  void handle_event_(ModemEvent event);
  void run_tick_for_state_();
  void enqueue_event_(ModemEvent event);
  bool is_network_attached_();
  const char *get_state_();
  void set_state_(ModemComponentState state);
  const char *state_to_string_(ModemComponentState state);

  std::shared_ptr<esp_modem::DTE> dte_{nullptr};
  std::unique_ptr<esp_modem::DCE> dce_{nullptr};
  ModemType type_{MODEM_TYPE_UNKNOWN};
  InternalGPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *power_pin_{nullptr};
  InternalGPIOPin *pwrkey_pin_{nullptr};
  uint8_t tx_pin_{0};
  uint8_t rx_pin_{0};
  std::string apn_{""};
  std::string use_address_;
  mutable std::string default_use_address_;
  int uart_event_task_stack_size_{0};
  int uart_event_task_priority_{0};
  int uart_event_queue_size_{0};
  int uart_tx_buffer_size_{0};
  int uart_rx_buffer_size_{0};
  uint32_t pull_time_{0};
  uint32_t state_entered_at_ms_{0};
  LockFreeQueue<ModemQueuedEvent, 9> event_queue_{};

  ModemComponentState state_{ModemComponentState::TURNING_ON_POWER};
  int connect_begin_;
  esp_netif_t *modem_netif_{nullptr};
  // esp_eth_phy_t *phy_{nullptr};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern ModemComponent *global_modem_component;

}  // namespace modem
}  // namespace esphome

#endif  // USE_ESP_IDF
#endif  // USE_ESP32
