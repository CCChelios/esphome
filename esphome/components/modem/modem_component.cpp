#include "modem_component.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/application.h"


#ifdef USE_ESP32

#include "esp_modem_c_api_types.h"
#include "esp_netif_ppp.h"
#include "cxx_include/esp_modem_types.hpp"

#include <cinttypes>
#include "driver/gpio.h"
#include <lwip/dns.h>
#include "esp_event.h"

uint32_t time_info_print = 0;
uint32_t time_hard_reset_modem = 0;
uint32_t time_check_rssi = 0;
uint32_t time_push_pwrkey = 0;
uint32_t time_turn_on_modem = 0;

#define TIME_TO_NEXT_HARD_RESET 30000
#define TIME_TO_START_MODEM 9000
#define TIME_CHECK_REGISTRATION_IN_NETWORK 1000

namespace esphome {
namespace modem {

static const char *const TAG = "modem";

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
  esp_log_level_set("esp-netif_lwip-ppp", ESP_LOG_VERBOSE);
  esp_log_level_set("esp-netif_lwip", ESP_LOG_VERBOSE);
  esp_log_level_set("modem", ESP_LOG_VERBOSE);
  esp_log_level_set("mqtt", ESP_LOG_VERBOSE);
  esp_log_level_set("command_lib", ESP_LOG_VERBOSE);
  esp_log_level_set("uart_terminal", ESP_LOG_VERBOSE);

  ESP_LOGCONFIG(TAG, "Setting up modem...");

  if (this->power_pin_) {
    this->power_pin_->setup();
  }
  if (this->pwrkey_pin_) {
    this->pwrkey_pin_->setup();
  }
  this->reset_pin_->setup();

  this->turn_on_modem();
  this->use_pwrkey();
  // esp_modem_hard_reset();

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    // Delay here to allow power to stabilise before Modem is initialized.
    delay(300);  // NOLINT
  }

  esp_err_t err;
  err = esp_netif_init();
  ESPHL_ERROR_CHECK(err, "modem netif init error");
  err = esp_event_loop_create_default();
  ESPHL_ERROR_CHECK(err, "modem event loop error");
  ESP_LOGCONFIG(TAG, "Initing netif");

  esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ModemComponent::got_ip_event_handler, NULL);

  /* Configure the PPP netif */
  esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(this->apn_.c_str());
  esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
  this->modem_netif_ = esp_netif_new(&netif_ppp_config);
  assert(this->modem_netif_);
  ESP_LOGD(TAG, "netif create succes");

  /* Configure the DTE */
  esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
  /* setup UART specific configuration based on kconfig options */
  dte_config.uart_config.tx_io_num = this->tx_pin_;
  dte_config.uart_config.rx_io_num = this->rx_pin_;
  dte_config.uart_config.rx_buffer_size = this->uart_rx_buffer_size_;
  dte_config.uart_config.tx_buffer_size = this->uart_tx_buffer_size_;
  dte_config.uart_config.event_queue_size = this->uart_event_queue_size_;
  dte_config.task_stack_size = this->uart_event_task_stack_size_;
  dte_config.task_priority = this->uart_event_task_priority_;
  dte_config.dte_buffer_size = this->uart_rx_buffer_size_ / 2;

  ESP_LOGD(TAG, "Initializing esp_modem");
  this->dte = esp_modem::create_uart_dte(&dte_config);
  this->dce = esp_modem::create_SIM800_dce(&dce_config, dte, this->modem_netif_);

  esp_netif_flags_t flags = esp_netif_get_flags(this->modem_netif_);

  this->started_ = true;
}

void ModemComponent::loop() {
  const uint32_t now = millis();
  switch (this->state_) {
    case ModemComponentState::STOPPED:
      this->turn_on_modem();
      // if (time_check_rssi + TIME_TO_START_MODEM < now) {
      //   time_check_rssi = now;
      //   // this->dce->set_command_mode();
      //   if (get_rssi()) {
      //     ESP_LOGD(TAG, "Starting modem connection");
      //     ESP_LOGD(TAG, "SIgnal quality: rssi=%d", get_rssi());
      //     this->state_ = ModemComponentState::CONNECTING;
      //     this->dce->set_data();
      //     // this->start_connect_();
      //   }
      //   if (time_hard_reset_modem + TIME_TO_NEXT_HARD_RESET < now) {
      //     time_hard_reset_modem = now;
      //     reset_modem();
      //   }
      // }
      break;
    case ModemComponentState::TURNING_ON:
      if (this->dce->sync() != esp_modem::command_result::OK){
        ESP_LOGD(TAG, "sync OK");
        this->state_ = ModemComponentState::REGISTRATION_IN_NETWORK;
      }
      break;
    case ModemComponentState::REGISTRATION_IN_NETWORK:
      if (time_check_rssi + TIME_CHECK_REGISTRATION_IN_NETWORK < now){
        time_check_rssi = now;
        if (get_rssi()) {
          ESP_LOGD(TAG, "Starting modem connection");
          ESP_LOGD(TAG, "SIgnal quality: rssi=%d", get_rssi());
          this->state_ = ModemComponentState::CONNECTING;
          this->dce->set_data();
          // this->start_connect_();
        } else {
          ESP_LOGD(TAG, "Wait RSSI");
        }
      }
      break;
    case ModemComponentState::CONNECTING:
      break;
    case ModemComponentState::CONNECTED:
      if (time_info_print < now) {
        // ESP_LOGI(TAG, "voltage %dV.", get_modem_voltage() / 1000);
        if (esp_netif_is_netif_up(this->modem_netif_)) {
          
          ESP_LOGD(TAG, "esp_netif_is_netif_UP");
        } else {
          ESP_LOGD(TAG, "esp_netif_is_netif_DOWN");
        }
        time_info_print = now + 5000;
      }
      break;
    case ModemComponentState::RESETTING:
      break;
    case ModemComponentState::TURNING_OFF:
      break;
  }
}

bool ModemComponent::turn_on_modem() {
  if (this->power_pin_) {
    this->power_pin_->digital_write(true);
    time_turn_on_modem = millis();
    vTaskDelay(pdMS_TO_TICKS(1900));  // NOLINT
    this->state_ = ModemComponentState::TURNING_ON;
    ESP_LOGD(TAG, "modem is on");
    return true;
  } else {
    ESP_LOGD(TAG, "failed to turn on modem because power_pin_ is not initialized");
    return false;
  }
  // wait no more than 1.9 sec for signs of life to appear
}

bool ModemComponent::turn_off_modem() {
  if (this->power_pin_) {
    this->power_pin_->digital_write(true);
    vTaskDelay(pdMS_TO_TICKS(1900));  // NOLINT
    ESP_LOGD(TAG, "modem is off");
    return true;
  } else {
    ESP_LOGD(TAG, "failed to turn off modem because power_pin_ is not initialized");
    return false;
  }
  // wait no more than 1.9 sec for signs of life to appear
}

bool ModemComponent::use_pwrkey() {
  if (this->pwrkey_pin_) {
    ESP_LOGD(TAG, "pwrkey used");
    this->pwrkey_pin_->digital_write(false);
    vTaskDelay(pdMS_TO_TICKS(1050));  // NOLINT
    this->pwrkey_pin_->digital_write(true);
  } else {
    ESP_LOGD(TAG, "failed to press button because pwrkey_pin_ is not initialized");
  }
  return true;
}

void ModemComponent::reset_modem() {
  ESP_LOGD(TAG, "reset modem");
  this->reset_pin_->digital_write(false);
  vTaskDelay(pdMS_TO_TICKS(105));  // NOLINT
  this->reset_pin_->digital_write(true);
  time_hard_reset_modem = millis();
  // you need to wait another 2,7 sec to get the status
}
void ModemComponent::dump_config() {
  this->dump_connect_params();
  ESP_LOGCONFIG(TAG, "Modem:");
  ESP_LOGCONFIG(TAG, "  Type: %d", this->type_);
  ESP_LOGCONFIG(TAG, "  Reset Pin: %u", this->reset_pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  Power pin : %s", (this->power_pin_) ? this->power_pin_->dump_summary().c_str() : "Not defined");
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
  int rssi = 0, ber = 0;
  esp_modem::command_result errr = this->dce->get_signal_quality(rssi, ber);
  // esp_err_t err = esp_modem::esp_modem_get_signal_quality(dce, &rssi, &ber);
  if (errr != esp_modem::command_result::OK) {
    ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with");
  }
  return rssi;
}

int ModemComponent::get_modem_voltage() {
  int voltage = 0, bcs = 0, bcl = 0;
  esp_modem::command_result errr = this->dce->get_battery_status(voltage, bcs, bcl);
  if (errr != esp_modem::command_result::OK) {
    ESP_LOGE(TAG, "get_battery_status failed with");
  }
  return voltage;
}

float ModemComponent::get_setup_priority() const { return setup_priority::WIFI; }

bool ModemComponent::can_proceed() { return this->is_connected(); }

network::IPAddress ModemComponent::get_ip_address() {
  esp_netif_ip_info_t ip;
  esp_netif_get_ip_info(this->modem_netif_, &ip);
  return network::IPAddress(&ip.ip);
}

void ModemComponent::got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "IP event! %" PRIu32, event_id);
  if (event_id == IP_EVENT_PPP_GOT_IP) {
    global_modem_component->connected_ = true;
    global_modem_component->state_ = ModemComponentState::CONNECTED;
    esp_netif_dns_info_t dns_info;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    esp_netif_t *netif = event->esp_netif;

    ESP_LOGI(TAG, "Modem Connect to PPP Server");
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
    ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info);
    ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");

    ESP_LOGD(TAG, "GOT ip event!!!");
  } else if (event_id == IP_EVENT_PPP_LOST_IP) {
    ESP_LOGD(TAG, "Modem Disconnect from PPP Server");
    global_modem_component->state_ = ModemComponentState::STOPPED;
  }
}

void ModemComponent::start_connect_() {
  this->connect_begin_ = millis();
  this->status_set_warning();
  reset_modem();
  esp_err_t err;
  err = esp_netif_set_hostname(this->modem_netif_, App.get_name().c_str());
  if (err != ERR_OK) {
    ESP_LOGD(TAG, "esp_netif_set_hostname failed: %s", esp_err_to_name(err));
  }

  // esp_netif_dhcp_status_t status = ESP_NETIF_DHCP_INIT;

  // restart ppp connection
  this->dce->exit_data();
  int rssi, ber;
  esp_modem::command_result errr = this->dce->get_signal_quality(rssi, ber);
  // esp_err_t err = esp_modem::esp_modem_get_signal_quality(dce, &rssi, &ber);
  if (errr != esp_modem::command_result::OK) {
    ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with");
    return;
  }
  ESP_LOGD(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);
  this->dce->set_data();

  //  this->status_set_warning();
}

bool ModemComponent::is_connected() { return this->state_ == ModemComponentState::CONNECTED; }
void ModemComponent::set_power_pin(InternalGPIOPin *power_pin) { this->power_pin_ = power_pin; }
void ModemComponent::set_pwrkey_pin(InternalGPIOPin *pwrkey_pin) { this->pwrkey_pin_ = pwrkey_pin; }
void ModemComponent::set_type(ModemType type) { this->type_ = type; }
void ModemComponent::set_reset_pin(InternalGPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
void ModemComponent::set_apn(const std::string &apn) { this->apn_ = apn; }
void ModemComponent::set_tx_pin(int tx_pin) { this->tx_pin_ = tx_pin; }
void ModemComponent::set_rx_pin(int rx_pin) { this->rx_pin_ = rx_pin; }
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

std::string ModemComponent::get_use_address() const {
  if (this->use_address_.empty()) {
    return App.get_name() + ".local";
  }
  return this->use_address_;
}

void ModemComponent::set_use_address(const std::string &use_address) { this->use_address_ = use_address; }

}  // namespace modem
}  // namespace esphome

#endif  // USE_ESP32