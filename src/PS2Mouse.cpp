#include "PS2Mouse.hpp"

namespace esp32_ps2dev {

PS2Mouse::PS2Mouse(int clk, int data) : PS2dev(clk, data) {}
void PS2Mouse::begin() {
  PS2dev::begin();

  xSemaphoreTake(_mutex_bus, portMAX_DELAY);
  delayMicroseconds(BYTE_INTERVAL_MICROS);
  while (write(0xAA) != 0) delay(200);
  delayMicroseconds(BYTE_INTERVAL_MICROS);
  while (write(0x00) != 0) delay(1);
  xSemaphoreGive(_mutex_bus);

  xTaskCreateUniversal(_taskfn_poll_mouse_count, "PS2Mouse", 4096, this, _config_task_priority - 1, &_task_poll_mouse_count,
                       _config_task_core);
}
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
}
uint8_t PS2Mouse::get_sample_rate() { return _sample_rate; }
void PS2Mouse::move(int16_t x, int16_t y, int8_t wheel) {
  _count_x += x;
  _count_y += y;
  _count_z += wheel;
  xTaskNotifyGive(_task_poll_mouse_count);
}
void PS2Mouse::press(Button button) {
  switch (button) {
    case Button::LEFT:
      _button_left = 1;
      break;
    case Button::RIGHT:
      _button_right = 1;
      break;
    case Button::MIDDLE:
      _button_middle = 1;
      break;
    case Button::BUTTON_4:
      _button_4th = 1;
      break;
    case Button::BUTTON_5:
      _button_5th = 1;
      break;
    default:
      break;
  }
  xTaskNotifyGive(_task_poll_mouse_count);
}
void PS2Mouse::release(Button button) {
  switch (button) {
    case Button::LEFT:
      _button_left = 0;
      break;
    case Button::RIGHT:
      _button_right = 0;
      break;
    case Button::MIDDLE:
      _button_middle = 0;
      break;
    case Button::BUTTON_4:
      _button_4th = 0;
      break;
    case Button::BUTTON_5:
      _button_5th = 0;
      break;
    default:
      break;
  }
  xTaskNotifyGive(_task_poll_mouse_count);
}
void PS2Mouse::click(Button button) {
  press(button);
  delay(MOUSE_CLICK_PRESSING_DURATION_MILLIS);
  release(button);
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
  if (_count_x > 255) {
    _count_x_overflow = 1;
    _count_x = 255;
  } else if (_count_x < -255) {
    _count_x_overflow = 1;
    _count_x = -255;
  }
  if (_count_y > 255) {
    _count_y_overflow = 1;
    _count_y = 255;
  } else if (_count_y < -255) {
    _count_y_overflow = 1;
    _count_y = -255;
  }
  if (_count_z > 7) {
    _count_z = 7;
  } else if (_count_z < -8) {
    _count_z = -8;
  }

  packet.len = 3 + _has_wheel;
  packet.data[0] = (_button_left) | ((_button_right) << 1) | ((_button_middle) << 2) | (1 << 3) | ((_count_x < 0) << 4) |
                   ((_count_y < 0) << 5) | (_count_x_overflow << 6) | (_count_y_overflow << 7);
  packet.data[1] = _count_x & 0xFF;
  packet.data[2] = _count_y & 0xFF;
  if (_has_wheel && !_has_4th_and_5th_buttons) {
    packet.data[3] = _count_z & 0xFF;
  } else if (_has_wheel && _has_4th_and_5th_buttons) {
    packet.data[3] = (_count_z & 0x0F) | ((_button_4th) << 4) | ((_button_5th) << 5);
  }

  send_packet(packet);
  reset_counter();
}
void PS2Mouse::_send_status() {
  PS2Packet packet;
  packet.len = 3;
  boolean mode = (_mode == Mode::REMOTE_MODE);
  packet.data[0] = (_button_right & 1) & ((_button_middle & 1) << 1) & ((_button_left & 1) << 2) & ((0) << 3) &
                   (((uint8_t)_scale & 1) << 4) & ((_data_reporting_enabled & 1) << 5) & ((mode & 1) << 6) & ((0) << 7);
  packet.data[1] = (uint8_t)_resolution;
  packet.data[2] = _sample_rate;
  send_packet(packet);
}

void _taskfn_poll_mouse_count(void* arg) {
  PS2Mouse* ps2mouse = (PS2Mouse*)arg;
  while (true) {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    if (ps2mouse->data_reporting_enabled()) {
      ps2mouse->_report();
    }
    delay(1000 / ps2mouse->get_sample_rate());
  }
  vTaskDelete(NULL);
}

}  // namespace esp32_ps2dev