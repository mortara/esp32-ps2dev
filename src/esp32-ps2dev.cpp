#include "esp32-ps2dev.h"

namespace esp32_ps2dev {

PS2dev::PS2dev(int clk, int data) {
  _ps2clk = clk;
  _ps2data = data;
  gohi(_ps2clk);
  gohi(_ps2data);
  _mutex_bus = xSemaphoreCreateMutex();
  _queue_packet = xQueueCreate(PACKET_QUEUE_LENGTH, sizeof(PS2Packet));
  xTaskCreateUniversal(_taskfn_process_host_request, "process_host_request", 4096, this, 1, &_task_process_host_request, APP_CPU_NUM);
  xTaskCreateUniversal(_taskfn_send_packet, "send_packet", 4096, this, 0, &_task_send_packet, APP_CPU_NUM);
}

void PS2dev::gohi(int pin) {
  digitalWrite(pin, HIGH);
  pinMode(pin, INPUT);
}
void PS2dev::golo(int pin) {
  pinMode(pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(pin, LOW);
}
void PS2dev::ack() {
  delayMicroseconds(BYTE_INTERVAL_MICROS);
  write(0xFA);
  delayMicroseconds(BYTE_INTERVAL_MICROS);
}
int PS2dev::write(unsigned char data) {
  unsigned char i;
  unsigned char parity = 1;

  if (get_bus_state() == BusState::COMMUNICATION_INHIBITED) {
    return -1;
  }

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);

  golo(_ps2data);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  // device sends on falling clock
  golo(_ps2clk);  // start bit
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

  for (i = 0; i < 8; i++) {
    if (data & 0x01) {
      gohi(_ps2data);
    } else {
      golo(_ps2data);
    }
    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
    golo(_ps2clk);
    delayMicroseconds(CLK_HALF_PERIOD_MICROS);
    gohi(_ps2clk);
    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }
  // parity bit
  if (parity) {
    gohi(_ps2data);
  } else {
    golo(_ps2data);
  }
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  golo(_ps2clk);
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

  // stop bit
  gohi(_ps2data);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  golo(_ps2clk);
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

  taskEXIT_CRITICAL(&mux);

  return 0;
}
int PS2dev::write_wait_idle(uint8_t data, uint64_t timeout_micros) {
  uint64_t start_time = micros();
  while (get_bus_state() != BusState::IDLE) {
    if (micros() - start_time > timeout_micros) {
      return -1;
    }
  }
  return write(data);
}
int PS2dev::read(unsigned char* value, uint64_t timeout_ms) {
  unsigned int data = 0x00;
  unsigned int bit = 0x01;

  unsigned char calculated_parity = 1;
  unsigned char received_parity = 0;

  // wait for data line to go low and clock line to go high (or timeout)
  unsigned long waiting_since = millis();
  while (get_bus_state() != BusState::HOST_REQUEST_TO_SEND) {
    if ((millis() - waiting_since) > timeout_ms) return -1;
    delay(1);
  }

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);

  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  golo(_ps2clk);
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

  while (bit < 0x0100) {
    if (digitalRead(_ps2data) == HIGH) {
      data = data | bit;
      calculated_parity = calculated_parity ^ 1;
    } else {
      calculated_parity = calculated_parity ^ 0;
    }

    bit = bit << 1;

    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
    golo(_ps2clk);
    delayMicroseconds(CLK_HALF_PERIOD_MICROS);
    gohi(_ps2clk);
    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  }
  // we do the delay at the end of the loop, so at this point we have
  // already done the delay for the parity bit

  // parity bit
  if (digitalRead(_ps2data) == HIGH) {
    received_parity = 1;
  }

  // stop bit
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  golo(_ps2clk);
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);

  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  golo(_ps2data);
  golo(_ps2clk);
  delayMicroseconds(CLK_HALF_PERIOD_MICROS);
  gohi(_ps2clk);
  delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  gohi(_ps2data);

  taskEXIT_CRITICAL(&mux);

  *value = data & 0x00FF;

  if (received_parity == calculated_parity) {
    return 0;
  } else {
    return -2;
  }
}
PS2dev::BusState PS2dev::get_bus_state() {
  if (digitalRead(_ps2clk) == LOW) {
    return BusState::COMMUNICATION_INHIBITED;
  } else if (digitalRead(_ps2data) == LOW) {
    return BusState::HOST_REQUEST_TO_SEND;
  } else {
    return BusState::IDLE;
  }
}
SemaphoreHandle_t PS2dev::get_bus_mutex_handle() { return _mutex_bus; }
QueueHandle_t PS2dev::get_packet_queue_handle() { return _queue_packet; }
int PS2dev::send_packet(PS2Packet* packet) { return (xQueueSend(_queue_packet, packet, 0) == pdTRUE) ? 0 : -1; }

PS2Mouse::PS2Mouse(int clk, int data) : PS2dev(clk, data) {}
int PS2Mouse::reply_to_host(uint8_t host_cmd) {
  uint8_t val;
  if (_mode == Mode::WRAP_MODE) {
    switch ((Command)host_cmd) {
      case Command::SET_WRAP_MODE:  // set wrap mode
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: (WRAP_MODE) Set wrap mode command received");
#endif
        ack();
        reset_counter();
        break;
      case Command::RESET_WRAP_MODE:  // reset wrap mode
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: (WRAP_MODE) Reset wrap mode command received");
#endif
        ack();
        reset_counter();
        _mode = _last_mode;
        break;
      default:
        write(host_cmd);
    }
    return 0;
  }

  switch ((Command)host_cmd) {
    case Command::RESET:  // reset
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Reset command received");
#endif
      ack();
      // the while loop lets us wait for the host to be ready
      while (write(0xAA) != 0) delay(1);
      while (write(0x00) != 0) delay(1);
      _has_wheel = false;
      _has_4th_and_5th_buttons = false;
      _sample_rate = 100;
      _resolution = ResolutionCode::RES_4;
      _scale = Scale::ONE_ONE;
      _data_reporting_enabled = false;
      _mode = Mode::STREAM_MODE;
      reset_counter();
      break;
    case Command::RESEND:  // resend
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Resend command received");
#endif
      ack();
      break;
    case Command::SET_DEFAULTS:  // set defaults
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set defaults command received");
#endif
      // enter stream mode
      ack();
      _sample_rate = 100;
      _resolution = ResolutionCode::RES_4;
      _scale = Scale::ONE_ONE;
      _data_reporting_enabled = false;
      _mode = Mode::STREAM_MODE;
      reset_counter();
      break;
    case Command::DISABLE_DATA_REPORTING:  // disable data reporting
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Disable data reporting command received");
#endif
      ack();
      _data_reporting_enabled = false;
      reset_counter();
      break;
    case Command::ENABLE_DATA_REPORTING:  // enable data reporting
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Enable data reporting command received");
#endif
      ack();
      _data_reporting_enabled = true;
      reset_counter();
      break;
    case Command::SET_SAMPLE_RATE:  // set sample rate
      ack();
      if (read(&val) == 0) {
        switch (val) {
          case 10:
          case 20:
          case 40:
          case 60:
          case 80:
          case 100:
          case 200:
            _sample_rate = val;
            _last_sample_rate[0] = _last_sample_rate[1];
            _last_sample_rate[1] = _last_sample_rate[2];
            _last_sample_rate[2] = val;
#if defined(_ESP32_PS2DEV_DEBUG_)
            _ESP32_PS2DEV_DEBUG_.print("Set sample rate command received: ");
            _ESP32_PS2DEV_DEBUG_.println(val);
#endif
            ack();
            break;

          default:
            break;
        }
        // _min_report_interval_us = 1000000 / sample_rate;
        reset_counter();
      }
      break;
    case Command::GET_DEVICE_ID:  // get device id
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Get device id command received");
#endif
      ack();
      if (_last_sample_rate[0] == 200 && _last_sample_rate[1] == 100 && _last_sample_rate[2] == 80) {
        write(0x03);  // Intellimouse with wheel
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Act as Intellimouse with wheel.");
#endif
        _has_wheel = true;
      } else if (_last_sample_rate[0] == 200 && _last_sample_rate[1] == 200 && _last_sample_rate[2] == 80 && _has_wheel == true) {
        write(0x04);  // Intellimouse with 4th and 5th buttons
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Act as Intellimouse with 4th and 5th buttons.");
#endif
        _has_4th_and_5th_buttons = true;
      } else {
        write(0x00);  // Standard PS/2 mouse
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Act as standard PS/2 mouse.");
#endif
        _has_wheel = false;
        _has_4th_and_5th_buttons = false;
      }
      reset_counter();
      break;
    case Command::SET_REMOTE_MODE:  // set remote mode
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set remote mode command received");
#endif
      ack();
      reset_counter();
      _mode = Mode::REMOTE_MODE;
      break;
    case Command::SET_WRAP_MODE:  // set wrap mode
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set wrap mode command received");
#endif
      ack();
      reset_counter();
      _last_mode = _mode;
      _mode = Mode::WRAP_MODE;
      break;
    case Command::RESET_WRAP_MODE:  // reset wrap mode
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Reset wrap mode command received");
#endif
      ack();
      reset_counter();
      break;
    case Command::READ_DATA:  // read data
      ack();
      _report();
      reset_counter();
      break;
    case Command::SET_STREAM_MODE:  // set stream mode
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set stream mode command received");
#endif
      ack();
      reset_counter();
      break;
    case Command::STATUS_REQUEST:  // status request
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Status request command received");
#endif
      ack();
      _send_status();
      break;
    case Command::SET_RESOLUTION:  // set resolution
      ack();
      if (read(&val) == 0 && val <= 3) {
        _resolution = (ResolutionCode)val;
#if defined(_ESP32_PS2DEV_DEBUG_)
        _ESP32_PS2DEV_DEBUG_.print("PS2Mouse::reply_to_host: Set resolution command received: ");
        _ESP32_PS2DEV_DEBUG_.println(val, HEX);
#endif
        ack();
        reset_counter();
      }
      break;
    case Command::SET_SCALING_2_1:  // set scaling 2:1
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set scaling 2:1 command received");
#endif
      ack();
      _scale = Scale::TWO_ONE;
      break;
    case Command::SET_SCALING_1_1:  // set scaling 1:1
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Mouse::reply_to_host: Set scaling 1:1 command received");
#endif
      ack();
      _scale = Scale::ONE_ONE;
      break;
    default:
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.print("PS2Mouse::reply_to_host: Unknown command received: ");
      _ESP32_PS2DEV_DEBUG_.println(host_cmd, HEX);
#endif
      break;
  }
  return 0;
}
bool PS2Mouse::has_wheel() { return _has_wheel; }
bool PS2Mouse::has_4th_and_5th_buttons() { return _has_4th_and_5th_buttons; }
bool PS2Mouse::data_reporting_enabled() { return _data_reporting_enabled; }
void PS2Mouse::reset_counter() {
  _count_x = 0;
  _count_y = 0;
  _count_z = 0;
  _count_x_overflow = 0;
  _count_y_overflow = 0;
  _count_or_button_changed = false;
}
void PS2Mouse::_send_status() {
  uint8_t data[3];
  boolean mode = (_mode == Mode::REMOTE_MODE);
  data[0] = (_button_right & 1) & ((_button_middle & 1) << 1) & ((_button_left & 1) << 2) & ((0) << 3) & (((uint8_t)_scale & 1) << 4) &
            ((_data_reporting_enabled & 1) << 5) & ((mode & 1) << 6) & ((0) << 7);
  data[1] = (uint8_t)_resolution;
  data[2] = _sample_rate;
  for (size_t i = 0; i < 3; i++) {
    write(data[i]);
  }
}
void PS2Mouse::_report() {
  PS2Packet packet;
  if (_scale == Scale::TWO_ONE) {
    int16_t* p[2] = {&_count_x, &_count_y};
    for (size_t i = 0; i < 2; i++) {
      boolean positive = *p[i] >= 0;
      uint16_t abs_value = positive ? *p[i] : -*p[i];
      switch (abs_value) {
        case 1:
          abs_value = 1;
          break;
        case 2:
          abs_value = 1;
          break;
        case 3:
          abs_value = 3;
          break;
        case 4:
          abs_value = 6;
          break;
        case 5:
          abs_value = 9;
          break;
        default:
          abs_value *= 2;
          break;
      }
      if (!positive) *p[i] = -abs_value;
    }
  }
  packet.len = 3 + _has_wheel;
  packet.data[0] = (_button_left) & ((_button_right) << 1) & ((_button_middle) << 2) & (1 << 3) & (((_count_x & 0x0100) >> 8) << 4) &
                   (((_count_y & 0x0100) >> 8) << 5) & (_count_x_overflow << 6) & (_count_y_overflow << 7);
  packet.data[1] = _count_x & 0xFF;
  packet.data[2] = _count_y & 0xFF;
  if (_has_wheel && !_has_4th_and_5th_buttons) {
    packet.data[3] = _count_z & 0xFF;
  } else if (_has_wheel && _has_4th_and_5th_buttons) {
    packet.data[3] = (_count_z & 0x0F) & ((_button_4th) << 4) & ((_button_5th) << 5);
  }
  send_packet(&packet);
  reset_counter();
}

PS2Keyboard::PS2Keyboard(int clk, int data) : PS2dev(clk, data) {}
bool PS2Keyboard::data_reporting_enabled() { return _data_reporting_enabled; }
bool PS2Keyboard::is_scroll_lock_led_on() { return _led_scroll_lock; }
bool PS2Keyboard::is_num_lock_led_on() { return _led_num_lock; }
bool PS2Keyboard::is_caps_lock_led_on() { return _led_caps_lock; }
int PS2Keyboard::reply_to_host(uint8_t host_cmd) {
  uint8_t val;
  switch ((Command)host_cmd) {
    case Command::RESET:  // reset
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Reset command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      // the while loop lets us wait for the host to be ready
      while (write((uint8_t)Command::ACK) != 0) delay(1);
      while (write((uint8_t)Command::BAT_SUCCESS) != 0) delay(1);
      _data_reporting_enabled = false;
      break;
    case Command::RESEND:  // resend
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Resend command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      ack();
      break;
    case Command::SET_DEFAULTS:  // set defaults
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Set defaults command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      // enter stream mode
      ack();
      break;
    case Command::DISABLE_DATA_REPORTING:  // disable data reporting
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Disable data reporting command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      _data_reporting_enabled = false;
      ack();
      break;
    case Command::ENABLE_DATA_REPORTING:  // enable data reporting
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Enable data reporting command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      _data_reporting_enabled = true;
      ack();
      break;
    case Command::SET_TYPEMATIC_RATE:  // set typematic rate
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Set typematic rate command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      ack();
      if (!read(&val)) ack();  // do nothing with the rate
      break;
    case Command::GET_DEVICE_ID:  // get device id
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Get device id command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      ack();
      write(0xAB);
      delayMicroseconds(BYTE_INTERVAL_MICROS);
      write(0x83);
      delayMicroseconds(BYTE_INTERVAL_MICROS);
      break;
    case Command::SET_SCAN_CODE_SET:  // set scan code set
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Set scan code set command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      ack();
      if (!read(&val)) ack();  // do nothing with the rate
      break;
    case Command::ECHO:  // echo
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Echo command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      delayMicroseconds(BYTE_INTERVAL_MICROS);
      write(0xEE);
      delayMicroseconds(BYTE_INTERVAL_MICROS);
      break;
    case Command::SET_RESET_LEDS:  // set/reset LEDs
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.println("PS2Keyboard::reply_to_host: Set/reset LEDs command received");
#endif  // _ESP32_PS2DEV_DEBUG_
      ack();
      if (!read(&val)) {
        ack();
        _led_scroll_lock = ((val & 1) != 0);
        _led_num_lock = ((val & 2) != 0);
        _led_caps_lock = ((val & 4) != 0);
      }
      return 1;
      break;
    default:
#if defined(_ESP32_PS2DEV_DEBUG_)
      _ESP32_PS2DEV_DEBUG_.print("PS2Keyboard::reply_to_host: Unknown command received: ");
      _ESP32_PS2DEV_DEBUG_.println(host_cmd, HEX);
#endif  // _ESP32_PS2DEV_DEBUG_
      break;
  }

  return 0;
}

void _taskfn_process_host_request(void* arg) {
  PS2dev* ps2dev = (PS2dev*)arg;
  while (true) {
    xSemaphoreTake(ps2dev->get_bus_mutex_handle(), portMAX_DELAY);
    if (ps2dev->get_bus_state() == PS2dev::BusState::HOST_REQUEST_TO_SEND) {
      uint8_t host_cmd;
      if (ps2dev->read(&host_cmd) == 0) {
        ps2dev->reply_to_host(host_cmd);
      }
    }
    xSemaphoreGive(ps2dev->get_bus_mutex_handle());
    delay(8);
  }
  vTaskDelete(NULL);
}
void _taskfn_send_packet(void* arg) {
  PS2dev* ps2dev = (PS2dev*)arg;
  while (true) {
    PS2Packet packet;
    if (xQueueReceive(ps2dev->get_packet_queue_handle(), &packet, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(ps2dev->get_bus_mutex_handle(), portMAX_DELAY);
      delayMicroseconds(BYTE_INTERVAL_MICROS);
      for (int i = 0; i < packet.len; i++) {
        ps2dev->write_wait_idle(packet.data[i]);
        delayMicroseconds(BYTE_INTERVAL_MICROS);
      }
      xSemaphoreGive(ps2dev->get_bus_mutex_handle());
    }
  }
  vTaskDelete(NULL);
}

}  // namespace esp32_ps2dev