#ifdef USE_ESP32
#ifdef USE_ESP_IDF

#include "modem_component.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/application.h"

#include "esp_modem_c_api_types.h"
#include "esp_netif_ppp.h"
#include "cxx_include/esp_modem_types.hpp"

#include <cinttypes>
#include <algorithm>
#include "driver/gpio.h"
#include <lwip/dns.h>
#include "esp_event.h"

namespace esphome {
namespace modem {

static const char *const TAG = "modem";
static constexpr uint32_t PWRKEY_CLICK_HOLD_MS = 2000;
static constexpr uint32_t RESET_CLICK_HOLD_MS = 2000;

ModemComponent *global_modem_component;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

#define ESPHL_ERROR_CHECK(err, message) \
  if ((err) != ESP_OK) { \
    ESP_LOGE(TAG, message ": (%d) %s", err, esp_err_to_name(err)); \
    this->mark_failed(); \
    return; \
  }

ModemComponent::ModemComponent() { global_modem_component = this; }

// setup
void ModemComponent::setup() {
  // esp_log_level_set("esp-netif_lwip-ppp", ESP_LOG_VERBOSE);
  // esp_log_level_set("esp-netif_lwip", ESP_LOG_VERBOSE);
  // esp_log_level_set("modem", ESP_LOG_VERBOSE);
  // esp_log_level_set("mqtt", ESP_LOG_VERBOSE);
  // esp_log_level_set("command_lib", ESP_LOG_VERBOSE);
  // esp_log_level_set("uart_terminal", ESP_LOG_VERBOSE);
  // esp_log_level_set("vfs_socket_creator", ESP_LOG_VERBOSE);
  // esp_log_level_set("vfs_uart_creator", ESP_LOG_VERBOSE);
  // esp_log_level_set("fs_terminal", ESP_LOG_VERBOSE);

  ESP_LOGCONFIG(TAG, "Setting up modem...");
  if (this->power_pin_) {
    this->power_pin_->setup();
  }
  if (this->pwrkey_pin_) {
    this->pwrkey_pin_->setup();
  }
  if (this->reset_pin_) {
    this->reset_pin_->setup();
  }
  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason != ESP_RST_DEEPSLEEP) {
    // Delay here to allow power to stabilise before Modem is initialized.
    delay(300);  // NOLINT
  }
  esp_err_t err;
  err = esp_netif_init();
  ESPHL_ERROR_CHECK(err, "modem netif init error");
  err = esp_event_loop_create_default();
  ESPHL_ERROR_CHECK(err, "modem event loop error");
  ESP_LOGCONFIG(TAG, "Initializing netif");
  esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ModemComponent::got_ip_event_handler, nullptr);
  ESP_LOGD(TAG, "Initializing esp_modem");
  this->modem_netif_init_();
  this->dte_init_();
}

void ModemComponent::loop() {
  this->schedule_timing_event_();
  this->process_event_queue_();
}

void ModemComponent::modem_netif_init_() {
#ifndef ESP_NETIF_DEFAULT_PPP
#define ESP_NETIF_DEFAULT_PPP() \
  { .base = NULL, .driver = NULL, .stack = NULL }
#endif
  esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
  this->modem_netif_ = esp_netif_new(&netif_ppp_config);
  assert(this->modem_netif_);
  ESP_LOGD(TAG, "Netif created successfully");
}

void ModemComponent::dte_init_() {
  constexpr int MIN_UART_TASK_STACK_SIZE = 4096;
  esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
  /* setup UART specific configuration based on kconfig options */
  dte_config.uart_config.tx_io_num = this->tx_pin_;
  dte_config.uart_config.rx_io_num = this->rx_pin_;
  dte_config.uart_config.rx_buffer_size = this->uart_rx_buffer_size_;
  dte_config.uart_config.tx_buffer_size = this->uart_tx_buffer_size_;
  dte_config.uart_config.event_queue_size = this->uart_event_queue_size_;
  dte_config.task_stack_size = std::max(this->uart_event_task_stack_size_, MIN_UART_TASK_STACK_SIZE);
  if (dte_config.task_stack_size != this->uart_event_task_stack_size_) {
    ESP_LOGW(TAG, "UART event task stack size %d is too low, using %d", this->uart_event_task_stack_size_,
             dte_config.task_stack_size);
  }
  dte_config.task_priority = this->uart_event_task_priority_;
  dte_config.dte_buffer_size = this->uart_rx_buffer_size_ / 2;
  this->dte_ = esp_modem::create_uart_dte(&dte_config);
}

void ModemComponent::dce_init_() {
  esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(this->apn_.c_str());
  this->dce_ = esp_modem::create_SIM800_dce(&dce_config, dte_, this->modem_netif_);
#ifdef CONFIG_ESP_MODEM_URC_HANDLER
  this->dte_->set_enhanced_urc_cb([this](const esp_modem::DTE::UrcBufferInfo &info) {
    if (info.new_data_size == 0) {
      return esp_modem::DTE::UrcConsumeInfo{esp_modem::DTE::UrcConsumeResult::CONSUME_NONE, 0};
    }
    if (info.new_data_start == nullptr) {
      ESP_LOGW(TAG, "URC!: new_data_start is null (size=%u)", static_cast<unsigned>(info.new_data_size));
      return esp_modem::DTE::UrcConsumeInfo{esp_modem::DTE::UrcConsumeResult::CONSUME_NONE, 0};
    }
    constexpr size_t MAX_TRACE_BYTES = 24;
    size_t trace_len = std::min(info.new_data_size, MAX_TRACE_BYTES);
    char ascii[MAX_TRACE_BYTES + 1];
    for (size_t i = 0; i < trace_len; i++) {
      uint8_t c = info.new_data_start[i];
      ascii[i] = ((c >= 0x20) && (c <= 0x7E)) ? static_cast<char>(c) : '.';
    }
    ascii[trace_len] = '\0';
    if (info.is_command_active) {
      ESP_LOGD(TAG, "CMD!: len=%u ascii=\"%s\"%s", static_cast<unsigned>(info.new_data_size), ascii,
               (info.new_data_size > trace_len) ? " ..." : "");
      return esp_modem::DTE::UrcConsumeInfo{esp_modem::DTE::UrcConsumeResult::CONSUME_NONE, 0};
    }
    ESP_LOGD(TAG, "URC!: len=%u ascii=\"%s\"%s", static_cast<unsigned>(info.new_data_size), ascii,
             (info.new_data_size > trace_len) ? " ..." : "");
    return esp_modem::DTE::UrcConsumeInfo{esp_modem::DTE::UrcConsumeResult::CONSUME_PARTIAL, info.new_data_size};
  });
#else
  ESP_LOGW(TAG, "URC handler is disabled (CONFIG_ESP_MODEM_URC_HANDLER=n)");
#endif
}

void ModemComponent::schedule_timing_event_() {
  const uint32_t now = millis();
  const ModemComponentStateTiming timing = this->get_state_timing_(this->state_);
  if (timing.time_limit && ((this->state_entered_at_ms_ + timing.time_limit) < now)) {
    this->enqueue_event_(ModemEvent::TIMEOUT);
    return;
  }
  if (!timing.poll_period) {
    this->enqueue_event_(ModemEvent::TICK);
    return;
  }
  if ((this->pull_time_ + timing.poll_period) < now) {
    this->pull_time_ = now;
    this->enqueue_event_(ModemEvent::TICK);
  }
}

void ModemComponent::process_event_queue_() {
  uint16_t dropped = this->event_queue_.get_and_reset_dropped_count();
  if (dropped > 0) {
    ESP_LOGW(TAG, "Dropped %u modem events due to buffer overflow", dropped);
  }

  ModemQueuedEvent *queued_event;
  while ((queued_event = this->event_queue_.pop()) != nullptr) {
    this->handle_event_(queued_event->event);
    delete queued_event;  // NOLINT(cppcoreguidelines-owning-memory)
  }
}

void ModemComponent::handle_event_(ModemEvent event) {
  switch (event) {
    case ModemEvent::TIMEOUT:
      if (this->state_ == ModemComponentState::SYNC && this->pwrkey_pin_ != nullptr) {
        ESP_LOGW(TAG, "State timeout in %s; toggling PWRKEY", this->state_to_string_(this->state_));
        this->set_state_(ModemComponentState::PWRKEY_CLICK);
        return;
      }
      ESP_LOGE(TAG, "State timeout in %s; switching to RESET_CLICK", this->state_to_string_(this->state_));
      this->set_state_(ModemComponentState::RESET_CLICK);
      return;
    case ModemEvent::PPP_LOST_IP:
      ESP_LOGD(TAG, "PPP lost IP; switching to RESET_CLICK");
      this->set_state_(ModemComponentState::RESET_CLICK);
      return;
    case ModemEvent::PPP_GOT_IP:
      ESP_LOGD(TAG, "PPP got IP; switching to CONNECTED");
      this->set_state_(ModemComponentState::CONNECTED);
      return;
    case ModemEvent::SYNC_OK:
      this->set_state_(ModemComponentState::REGISTRATION_IN_NETWORK);
      return;
    case ModemEvent::ATTACH_OK:
      if (!this->dce_->set_mode(esp_modem::modem_mode::CMUX_MODE)) {
        this->enqueue_event_(ModemEvent::CMUX_FAIL);
      } else {
        this->enqueue_event_(ModemEvent::CMUX_OK);
      }
      return;
    case ModemEvent::CMUX_OK:
      this->set_state_(ModemComponentState::CONNECTING);
      return;
    case ModemEvent::CMUX_FAIL:
      ESP_LOGW(TAG, "Failed to switch modem to CMUX mode; switching to RESET_CLICK");
      this->set_state_(ModemComponentState::RESET_CLICK);
      return;
    case ModemEvent::TICK:
      this->run_tick_for_state_();
      return;
    case ModemEvent::EVENT_COUNT:
      return;
  }
}

void ModemComponent::run_tick_for_state_() {
  switch (this->state_) {
    // The state of the beginning of power supply to the modem
    case ModemComponentState::TURNING_ON_POWER:
      if (this->power_pin_) {
        this->power_pin_->digital_write(true);
        ESP_LOGD(TAG, "Power on modem");
        if (this->pwrkey_pin_) {
          this->set_state_(ModemComponentState::PWRKEY_CLICK);
        } else {
          this->set_state_(ModemComponentState::SYNC);
        }
      } else if (this->pwrkey_pin_) {
        ESP_LOGD(TAG, "Power pin is not configured, but PWRKEY is configured; switching to PWRKEY_CLICK");
        this->set_state_(ModemComponentState::PWRKEY_CLICK);
      } else {
        ESP_LOGD(TAG, "Power pin and PWRKEY pin are not configured; switching to SYNC");
        this->set_state_(ModemComponentState::SYNC);
      }
      break;

    // Modem power supply end state
    case ModemComponentState::TURNING_OFF_POWER:
      if (this->power_pin_) {
        this->power_pin_->digital_write(false);
        ESP_LOGD(TAG, "Power off modem");
      } else {
        ESP_LOGD(TAG, "Power pin is not configured, skipping power off");
      }
      this->set_state_(ModemComponentState::TURNING_ON_POWER);
      break;

    // The state clicks the PWRKEY button
    case ModemComponentState::PWRKEY_CLICK:
      if (this->pwrkey_pin_ == nullptr) {
        ESP_LOGD(TAG, "PWRKEY pin is not configured; switching to SYNC");
        this->set_state_(ModemComponentState::SYNC);
        break;
      }
      if ((millis() - this->state_entered_at_ms_) >= PWRKEY_CLICK_HOLD_MS) {
        this->pwrkey_pin_->digital_write(true);
        ESP_LOGD(TAG, "Release PWRKEY");
        this->set_state_(ModemComponentState::SYNC);
      }
      break;

    // The state clicks the RESET button
    case ModemComponentState::RESET_CLICK:
      if (this->reset_pin_ == nullptr) {
        ESP_LOGD(TAG, "Reset pin is not configured; switching to TURNING_OFF_POWER");
        this->set_state_(ModemComponentState::TURNING_OFF_POWER);
        break;
      }
      if ((millis() - this->state_entered_at_ms_) >= RESET_CLICK_HOLD_MS) {
        this->reset_pin_->digital_write(true);
        ESP_LOGD(TAG, "Release reset pin");
        this->set_state_(ModemComponentState::SYNC);
      }
      break;

    // The state of waiting for the modem to connect, response to "AT" "OK"
    case ModemComponentState::SYNC:
      if (this->dce_->sync() == esp_modem::command_result::OK) {
        if (this->dce_->set_echo(false) != esp_modem::command_result::OK) {
          ESP_LOGD(TAG, "Failed to disable modem echo (ATE0)");
        }
        ESP_LOGD(TAG, "AT sync successful");
        this->enqueue_event_(ModemEvent::SYNC_OK);
      } else {
        ESP_LOGD(TAG, "Waiting for AT sync");
      }
      break;

    // The state of waiting for the modem to register in the network
    case ModemComponentState::REGISTRATION_IN_NETWORK:
      if (this->is_network_attached_()) {
        ESP_LOGD(TAG, "Modem is attached to network; starting data connection");
        this->enqueue_event_(ModemEvent::ATTACH_OK);
      } else {
        ESP_LOGD(TAG, "Waiting for modem network attachment");
      }
      break;

    // The state of waiting state for receiving IP address
    case ModemComponentState::CONNECTING:
      ESP_LOGD(TAG, "Waiting for PPP IP event");
      break;

    // The state of network connection established
    case ModemComponentState::CONNECTED:
      // ESP_LOGD(TAG, "The modem works!");
      break;
  }
}

void ModemComponent::enqueue_event_(ModemEvent event) {
  auto *queued_event = new ModemQueuedEvent;  // NOLINT(cppcoreguidelines-owning-memory)
  queued_event->event = event;
  if (!this->event_queue_.push(queued_event)) {
    delete queued_event;  // NOLINT(cppcoreguidelines-owning-memory)
  }
}

bool ModemComponent::is_network_attached_() {
  int attached = 0;
  esp_modem::command_result result = this->dce_->get_network_attachment_state(attached);
  if (result != esp_modem::command_result::OK) {
    ESP_LOGD(TAG, "Failed to read modem network attachment state");
    return false;
  }
  return attached == 1;
}

void ModemComponent::set_state_(ModemComponentState state) {
  // execute before transition to state
  switch (state) {
    case ModemComponentState::PWRKEY_CLICK:
      if (this->pwrkey_pin_ != nullptr) {
        this->pwrkey_pin_->digital_write(false);
        ESP_LOGD(TAG, "Assert PWRKEY");
      }
      break;
    case ModemComponentState::RESET_CLICK:
      if (this->reset_pin_ != nullptr) {
        this->reset_pin_->digital_write(false);
        ESP_LOGD(TAG, "Assert reset pin");
      }
      break;
    case ModemComponentState::SYNC:
      this->dce_init_();
      this->started = true;
      break;

    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "Modem component change state from %s to %s", this->state_to_string_(this->state_),
                this->state_to_string_(state));
  this->state_ = state;
  this->state_entered_at_ms_ = millis();
}

const char *ModemComponent::state_to_string_(ModemComponentState state) {
  switch (state) {
    case ModemComponentState::TURNING_ON_POWER:
      return "TURNING_ON_POWER";
    case ModemComponentState::TURNING_OFF_POWER:
      return "TURNING_OFF_POWER";
    case ModemComponentState::PWRKEY_CLICK:
      return "PWRKEY_CLICK";
    case ModemComponentState::RESET_CLICK:
      return "RESET_CLICK";
    case ModemComponentState::SYNC:
      return "SYNC";
    case ModemComponentState::REGISTRATION_IN_NETWORK:
      return "REGISTRATION_IN_NETWORK";
    case ModemComponentState::CONNECTING:
      return "CONNECTING";
    case ModemComponentState::CONNECTED:
      return "CONNECTED";
  }
  return "UNKNOWN";
}

void ModemComponent::dump_config() {
  this->dump_connect_params();
  ESP_LOGCONFIG(TAG, "Modem:");
  ESP_LOGCONFIG(TAG, "  Type: %d", this->type_);
  ESP_LOGCONFIG(TAG, "  Power pin : %s", (this->power_pin_) ? this->power_pin_->dump_summary().c_str() : "Not defined");
  ESP_LOGCONFIG(TAG, "  Reset pin : %s", (this->reset_pin_) ? this->reset_pin_->dump_summary().c_str() : "Not defined");
  ESP_LOGCONFIG(TAG, "  Pwrkey pin : %s",
                (this->pwrkey_pin_) ? this->pwrkey_pin_->dump_summary().c_str() : "Not defined");
  ESP_LOGCONFIG(TAG, "  APN: %s", this->apn_.c_str());
  ESP_LOGCONFIG(TAG, "  TX Pin: %d", this->tx_pin_);
  ESP_LOGCONFIG(TAG, "  RX Pin: %d", this->rx_pin_);
  ESP_LOGCONFIG(TAG, "  UART Event Task Stack Size: %d", this->uart_event_task_stack_size_);
  ESP_LOGCONFIG(TAG, "  UART Event Task Priority: %d", this->uart_event_task_priority_);
  ESP_LOGCONFIG(TAG, "  UART Event Queue Size: %d", this->uart_event_queue_size_);
  ESP_LOGCONFIG(TAG, "  UART TX Buffer Size: %d", this->uart_tx_buffer_size_);
  ESP_LOGCONFIG(TAG, "  UART RX Buffer Size: %d", this->uart_rx_buffer_size_);
}

void ModemComponent::dump_connect_params() {
  esp_netif_ip_info_t ip;
  esp_netif_get_ip_info(this->modem_netif_, &ip);
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::IPAddress(&ip.ip).str().c_str());
  ESP_LOGCONFIG(TAG, "  Netmask: %s", network::IPAddress(&ip.netmask).str().c_str());
  ESP_LOGCONFIG(TAG, "  Gateway: %s", network::IPAddress(&ip.gw).str().c_str());
  esp_netif_dns_info_t dns_info;
  esp_netif_get_dns_info(this->modem_netif_, ESP_NETIF_DNS_MAIN, &dns_info);
  ESP_LOGCONFIG(TAG, "  DNS1: %s", network::IPAddress(&dns_info.ip.u_addr.ip4).str().c_str());
  esp_netif_get_dns_info(this->modem_netif_, ESP_NETIF_DNS_BACKUP, &dns_info);
  ESP_LOGCONFIG(TAG, "  DNS2: %s", network::IPAddress(&dns_info.ip.u_addr.ip4).str().c_str());
}

int ModemComponent::get_rssi() {
  if (this->started) {
    int rssi = 0, ber = 0;
    esp_modem::command_result errr = this->dce_->get_signal_quality(rssi, ber);
    if (errr != esp_modem::command_result::OK) {
      ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with");
    }
    return rssi;
  }
  return 0;
}

int ModemComponent::get_ber() {
  if (this->started) {
    int rssi = 0, ber = 0;
    esp_modem::command_result errr = this->dce_->get_signal_quality(rssi, ber);
    if (errr != esp_modem::command_result::OK) {
      ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with");
    }
    return ber;
  }
  return 0;
}

int ModemComponent::get_modem_voltage() {
  if (this->started) {
    int milli_volt = 0, bcs = 0, bcl = 0;
    esp_modem::command_result errr = this->dce_->get_battery_status(milli_volt, bcs, bcl);
    if (errr != esp_modem::command_result::OK) {
      ESP_LOGE(TAG, "esp_modem_get_modem_voltage failed with");
    }
    return milli_volt;
  }
  return 0;
}

float ModemComponent::get_setup_priority() const { return setup_priority::MODEM; }

bool ModemComponent::can_proceed() { return this->is_connected(); }

network::IPAddress ModemComponent::get_ip_address() {
  esp_netif_ip_info_t ip;
  esp_netif_get_ip_info(this->modem_netif_, &ip);
  return network::IPAddress(&ip.ip);
}

network::IPAddresses ModemComponent::get_ip_addresses() {
  network::IPAddresses addresses;
  if (!this->is_connected()) {
    return addresses;
  }
  addresses[0] = this->get_ip_address();
  return addresses;
}

void ModemComponent::got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "IP event! %" PRIu32, event_id);
  if (event_id == IP_EVENT_PPP_GOT_IP) {
    if (global_modem_component != nullptr) {
      global_modem_component->enqueue_event_(ModemEvent::PPP_GOT_IP);
    }
    esp_netif_dns_info_t dns_info;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    esp_netif_t *netif = event->esp_netif;

    ESP_LOGI(TAG, "Modem Connect to PPP Server");
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
    ESP_LOGI(TAG, "DNS 1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info);
    ESP_LOGI(TAG, "DNS 2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");

    ESP_LOGD(TAG, "GOT ip event!!!");
  } else if (event_id == IP_EVENT_PPP_LOST_IP) {
    ESP_LOGD(TAG, "Modem Disconnect from PPP Server");
    if (global_modem_component != nullptr) {
      global_modem_component->enqueue_event_(ModemEvent::PPP_LOST_IP);
    }
  }
}

bool ModemComponent::is_connected() { return this->state_ == ModemComponentState::CONNECTED; }
bool ModemComponent::is_disabled() const { return false; }
void ModemComponent::set_power_pin(InternalGPIOPin *power_pin) { this->power_pin_ = power_pin; }
void ModemComponent::set_pwrkey_pin(InternalGPIOPin *pwrkey_pin) { this->pwrkey_pin_ = pwrkey_pin; }
void ModemComponent::set_type(ModemType type) { this->type_ = type; }
void ModemComponent::set_reset_pin(InternalGPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
void ModemComponent::set_apn(const std::string &apn) { this->apn_ = apn; }
void ModemComponent::set_tx_pin(uint8_t tx_pin) { this->tx_pin_ = tx_pin; }
void ModemComponent::set_rx_pin(uint8_t rx_pin) { this->rx_pin_ = rx_pin; }
void ModemComponent::set_uart_event_task_stack_size(int uart_event_task_stack_size) {
  this->uart_event_task_stack_size_ = uart_event_task_stack_size;
}
void ModemComponent::set_uart_event_task_priority(int uart_event_task_priority) {
  this->uart_event_task_priority_ = uart_event_task_priority;
}
void ModemComponent::set_uart_event_queue_size(int uart_event_queue_size) {
  this->uart_event_queue_size_ = uart_event_queue_size;
}
void ModemComponent::set_uart_tx_buffer_size(int uart_tx_buffer_size) {
  this->uart_tx_buffer_size_ = uart_tx_buffer_size;
}
void ModemComponent::set_uart_rx_buffer_size(int uart_rx_buffer_size) {
  this->uart_rx_buffer_size_ = uart_rx_buffer_size;
}

const char *ModemComponent::get_use_address() const {
  if (this->use_address_.empty()) {
    this->default_use_address_ = App.get_name().str() + ".local";
    return this->default_use_address_.c_str();
  }
  return this->use_address_.c_str();
}

void ModemComponent::set_use_address(const std::string &use_address) { this->use_address_ = use_address; }

}  // namespace modem
}  // namespace esphome

#endif  // USE_ESP_IDF
#endif  // USE_ESP32
