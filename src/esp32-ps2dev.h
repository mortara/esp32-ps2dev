#ifndef __ESP32_PS2DEV_H__
#define __ESP32_PS2DEV_H__

#include "Arduino.h"

namespace esp32_ps2dev {

const uint32_t CLK_HALF_PERIOD_MICROS = 40;
const uint32_t CLK_QUATER_PERIOD_MICROS = CLK_HALF_PERIOD_MICROS / 2;
const uint32_t BYTE_INTERVAL_MICROS = 500;
const int PACKET_QUEUE_LENGTH = 20;

class PS2Packet {
 public:
  uint8_t len;
  uint8_t data[16];
};

class PS2dev {
 public:
  PS2dev(int clk, int data);

  enum class BusState {
    IDLE,
    COMMUNICATION_INHIBITED,
    HOST_REQUEST_TO_SEND,
  };

  int write(unsigned char data);
  int write_wait_idle(uint8_t data, uint64_t timeout_micros = 1000);
  int read(unsigned char* data, uint64_t timeout_ms = 0);
  virtual int reply_to_host(uint8_t host_cmd) = 0;
  BusState get_bus_state();
  SemaphoreHandle_t get_bus_mutex_handle();
  QueueHandle_t get_packet_queue_handle();
  int send_packet(PS2Packet* packet);

 protected:
  int _ps2clk;
  int _ps2data;
  TaskHandle_t _task_process_host_request;
  TaskHandle_t _task_send_packet;
  QueueHandle_t _queue_packet;
  SemaphoreHandle_t _mutex_bus;
  void golo(int pin);
  void gohi(int pin);
  void ack();
};

class PS2Mouse : public PS2dev {
 public:
  PS2Mouse(int clk, int data);
  int reply_to_host(uint8_t host_cmd);
  bool has_wheel();
  bool has_4th_and_5th_buttons();
  bool data_reporting_enabled();
  void reset_counter();
  enum class ResolutionCode : uint8_t { RES_1 = 0x00, RES_2 = 0x01, RES_4 = 0x02, RES_8 = 0x03 };
  enum class Scale : uint8_t { ONE_ONE = 0, TWO_ONE = 1 };
  enum class Mode : uint8_t { REMOTE_MODE = 0, STREAM_MODE = 1, WRAP_MODE = 2 };
  enum class Command : uint8_t {
    RESET = 0xFF,
    RESEND = 0xFE,
    ERROR = 0xFC,
    ACK = 0xFA,
    SET_DEFAULTS = 0xF6,
    DISABLE_DATA_REPORTING = 0xF5,
    ENABLE_DATA_REPORTING = 0xF4,
    SET_SAMPLE_RATE = 0xF3,
    GET_DEVICE_ID = 0xF2,
    SET_REMOTE_MODE = 0xF0,
    SET_WRAP_MODE = 0xEE,
    RESET_WRAP_MODE = 0xEC,
    READ_DATA = 0xEB,
    SET_STREAM_MODE = 0xEA,
    STATUS_REQUEST = 0xE9,
    SET_RESOLUTION = 0xE8,
    SET_SCALING_2_1 = 0xE7,
    SET_SCALING_1_1 = 0xE6,
    SELF_TEST_PASSED = 0xAA,
  };

 protected:
  void _send_status();
  void _report();
  bool _has_wheel = false;
  bool _has_4th_and_5th_buttons = false;
  bool _data_reporting_enabled = false;
  ResolutionCode _resolution;
  Scale _scale;
  Mode _mode;
  Mode _last_mode;
  uint8_t _last_sample_rate[3] = {0, 0, 0};
  uint8_t _sample_rate;
  int16_t _count_x = 0;
  uint8_t _count_x_overflow = 0;
  int16_t _count_y = 0;
  uint8_t _count_y_overflow = 0;
  int8_t _count_z = 0;
  uint8_t _button_left = 0;
  uint8_t _button_right = 0;
  uint8_t _button_middle = 0;
  uint8_t _button_4th = 0;
  uint8_t _button_5th = 0;
  bool _count_or_button_changed = false;
};

class PS2Keyboard : public PS2dev {
 public:
  PS2Keyboard(int clk, int data);
  int reply_to_host(uint8_t host_cmd);
  enum class Command {
    RESET = 0xFF,
    RESEND = 0xFE,
    ACK = 0xFA,
    SET_DEFAULTS = 0xF6,
    DISABLE_DATA_REPORTING = 0xF5,
    ENABLE_DATA_REPORTING = 0xF4,
    SET_TYPEMATIC_RATE = 0xF3,
    GET_DEVICE_ID = 0xF2,
    SET_SCAN_CODE_SET = 0xF0,
    ECHO = 0xEE,
    SET_RESET_LEDS = 0xED,
    BAT_SUCCESS = 0xAA,
  };
  bool data_reporting_enabled();
  bool is_scroll_lock_led_on();
  bool is_num_lock_led_on();
  bool is_caps_lock_led_on();

 protected:
  bool _data_reporting_enabled = false;
  bool _led_scroll_lock = false;
  bool _led_num_lock = false;
  bool _led_caps_lock = false;
};

void _taskfn_process_host_request(void* arg);
void _taskfn_send_packet(void* arg);

}  // namespace esp32_ps2dev

#endif  // __ESP32_PS2DEV_H__