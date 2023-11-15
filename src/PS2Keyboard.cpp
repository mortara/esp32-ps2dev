#include "PS2Keyboard.hpp"

namespace esp32_ps2dev {

PS2Keyboard::PS2Keyboard(int clk, int data) : PS2dev(clk, data) {}
void PS2Keyboard::begin() {
  PS2dev::begin();

  xSemaphoreTake(_mutex_bus, portMAX_DELAY);
  delayMicroseconds(BYTE_INTERVAL_MICROS);
  delay(200);
  write(0xAA);
  xSemaphoreGive(_mutex_bus);
}
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
      ack();  // ack() provides delay, some systems need it
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
      while (write(0xAB) != 0) delay(1);  // ensure ID gets writed, some hosts may be sensitive
      while (write(0x83) != 0) delay(1);  // this is critical for combined ports (they decide mouse/kb on this)
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
      while (write(0xAF) != 0) delay(1);
      if (!read(&val)) {
        while (write(0xAF) != 0) delay(1);
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
void PS2Keyboard::keydown(scancodes::Key key) {
  if (!_data_reporting_enabled) return;
  PS2Packet packet;
  packet.len = scancodes::MAKE_CODES_LEN[key];
  for (uint8_t i = 0; i < packet.len; i++) {
    packet.data[i] = scancodes::MAKE_CODES[key][i];
  }
  send_packet(packet);
}
void PS2Keyboard::keyup(scancodes::Key key) {
  if (!_data_reporting_enabled) return;
  PS2Packet packet;
  packet.len = scancodes::BREAK_CODES_LEN[key];
  for (uint8_t i = 0; i < packet.len; i++) {
    packet.data[i] = scancodes::BREAK_CODES[key][i];
  }
  send_packet(packet);
}
void PS2Keyboard::type(scancodes::Key key) {
  keydown(key);
  delay(10);
  keyup(key);
}
void PS2Keyboard::type(std::initializer_list<scancodes::Key> keys) {
  std::stack<scancodes::Key> stack;
  for (auto key : keys) {
    keydown(key);
    stack.push(key);
    delay(10);
  }
  while (!stack.empty()) {
    keyup(stack.top());
    stack.pop();
    delay(10);
  }
}
void PS2Keyboard::type(const char* str) {
  size_t i = 0;
  while (str[i] != '\0') {
    char c = str[i];
    scancodes::Key key;
    bool shift = false;
    switch (c) {
      case '\b':
        key = scancodes::Key::K_BACKSPACE;
        break;
      case '\t':
        key = scancodes::Key::K_TAB;
        break;
      case '\r':
      case '\n':
        key = scancodes::Key::K_RETURN;
        break;
      case ' ':
        key = scancodes::Key::K_SPACE;
        break;
      case '!':
        shift = true;
        key = scancodes::Key::K_1;
        break;
      case '\"':
        shift = true;
        key = scancodes::Key::K_QUOTE;
        break;
      case '#':
        shift = true;
        key = scancodes::Key::K_3;
        break;
      case '$':
        shift = true;
        key = scancodes::Key::K_4;
        break;
      case '&':
        shift = true;
        key = scancodes::Key::K_7;
        break;
      case '\'':
        key = scancodes::Key::K_QUOTE;
        break;
      case '(':
        shift = true;
        key = scancodes::Key::K_9;
        break;
      case ')':
        shift = true;
        key = scancodes::Key::K_0;
        break;
      case '*':
        shift = true;
        key = scancodes::Key::K_8;
        break;
      case '+':
        shift = true;
        key = scancodes::Key::K_EQUALS;
        break;
      case ',':
        key = scancodes::Key::K_COMMA;
        break;
      case '-':
        key = scancodes::Key::K_MINUS;
        break;
      case '.':
        key = scancodes::Key::K_PERIOD;
        break;
      case '/':
        key = scancodes::Key::K_SLASH;
        break;
      case '0':
        key = scancodes::Key::K_0;
        break;
      case '1':
        key = scancodes::Key::K_1;
        break;
      case '2':
        key = scancodes::Key::K_2;
        break;
      case '3':
        key = scancodes::Key::K_3;
        break;
      case '4':
        key = scancodes::Key::K_4;
        break;
      case '5':
        key = scancodes::Key::K_5;
        break;
      case '6':
        key = scancodes::Key::K_6;
        break;
      case '7':
        key = scancodes::Key::K_7;
        break;
      case '8':
        key = scancodes::Key::K_8;
        break;
      case '9':
        key = scancodes::Key::K_9;
        break;
      case ':':
        shift = true;
        key = scancodes::Key::K_SEMICOLON;
        break;
      case ';':
        key = scancodes::Key::K_SEMICOLON;
        break;
      case '<':
        shift = true;
        key = scancodes::Key::K_COMMA;
        break;
      case '=':
        key = scancodes::Key::K_EQUALS;
        break;
      case '>':
        shift = true;
        key = scancodes::Key::K_PERIOD;
        break;
      case '\?':
        shift = true;
        key = scancodes::Key::K_SLASH;
        break;
      case '@':
        shift = true;
        key = scancodes::Key::K_2;
        break;
      case '[':
        key = scancodes::Key::K_LEFTBRACKET;
        break;
      case '\\':
        key = scancodes::Key::K_BACKSLASH;
        break;
      case ']':
        key = scancodes::Key::K_RIGHTBRACKET;
        break;
      case '^':
        shift = true;
        key = scancodes::Key::K_6;
        break;
      case '_':
        shift = true;
        key = scancodes::Key::K_MINUS;
        break;
      case '`':
        key = scancodes::Key::K_BACKQUOTE;
        break;
      case 'a':
        key = scancodes::Key::K_A;
        break;
      case 'b':
        key = scancodes::Key::K_B;
        break;
      case 'c':
        key = scancodes::Key::K_C;
        break;
      case 'd':
        key = scancodes::Key::K_D;
        break;
      case 'e':
        key = scancodes::Key::K_E;
        break;
      case 'f':
        key = scancodes::Key::K_F;
        break;
      case 'g':
        key = scancodes::Key::K_G;
        break;
      case 'h':
        key = scancodes::Key::K_H;
        break;
      case 'i':
        key = scancodes::Key::K_I;
        break;
      case 'j':
        key = scancodes::Key::K_J;
        break;
      case 'k':
        key = scancodes::Key::K_K;
        break;
      case 'l':
        key = scancodes::Key::K_L;
        break;
      case 'm':
        key = scancodes::Key::K_M;
        break;
      case 'n':
        key = scancodes::Key::K_N;
        break;
      case 'o':
        key = scancodes::Key::K_O;
        break;
      case 'p':
        key = scancodes::Key::K_P;
        break;
      case 'q':
        key = scancodes::Key::K_Q;
        break;
      case 'r':
        key = scancodes::Key::K_R;
        break;
      case 's':
        key = scancodes::Key::K_S;
        break;
      case 't':
        key = scancodes::Key::K_T;
        break;
      case 'u':
        key = scancodes::Key::K_U;
        break;
      case 'v':
        key = scancodes::Key::K_V;
        break;
      case 'w':
        key = scancodes::Key::K_W;
        break;
      case 'x':
        key = scancodes::Key::K_X;
        break;
      case 'y':
        key = scancodes::Key::K_Y;
        break;
      case 'z':
        key = scancodes::Key::K_Z;
        break;
      case 'A':
        shift = true;
        key = scancodes::Key::K_A;
        break;
      case 'B':
        shift = true;
        key = scancodes::Key::K_B;
        break;
      case 'C':
        shift = true;
        key = scancodes::Key::K_C;
        break;
      case 'D':
        shift = true;
        key = scancodes::Key::K_D;
        break;
      case 'E':
        shift = true;
        key = scancodes::Key::K_E;
        break;
      case 'F':
        shift = true;
        key = scancodes::Key::K_F;
        break;
      case 'G':
        shift = true;
        key = scancodes::Key::K_G;
        break;
      case 'H':
        shift = true;
        key = scancodes::Key::K_H;
        break;
      case 'I':
        shift = true;
        key = scancodes::Key::K_I;
        break;
      case 'J':
        shift = true;
        key = scancodes::Key::K_J;
        break;
      case 'K':
        shift = true;
        key = scancodes::Key::K_K;
        break;
      case 'L':
        shift = true;
        key = scancodes::Key::K_L;
        break;
      case 'M':
        shift = true;
        key = scancodes::Key::K_M;
        break;
      case 'N':
        shift = true;
        key = scancodes::Key::K_N;
        break;
      case 'O':
        shift = true;
        key = scancodes::Key::K_O;
        break;
      case 'P':
        shift = true;
        key = scancodes::Key::K_P;
        break;
      case 'Q':
        shift = true;
        key = scancodes::Key::K_Q;
        break;
      case 'R':
        shift = true;
        key = scancodes::Key::K_R;
        break;
      case 'S':
        shift = true;
        key = scancodes::Key::K_S;
        break;
      case 'T':
        shift = true;
        key = scancodes::Key::K_T;
        break;
      case 'U':
        shift = true;
        key = scancodes::Key::K_U;
        break;
      case 'V':
        shift = true;
        key = scancodes::Key::K_V;
        break;
      case 'W':
        shift = true;
        key = scancodes::Key::K_W;
        break;
      case 'X':
        shift = true;
        key = scancodes::Key::K_X;
        break;
      case 'Y':
        shift = true;
        key = scancodes::Key::K_Y;
        break;
      case 'Z':
        shift = true;
        key = scancodes::Key::K_Z;
        break;

      default:
        i++;
        continue;
        break;
    }
    if (shift) {
      keydown(scancodes::Key::K_LSHIFT);
      delay(10);
      type(key);
      delay(10);
      keyup(scancodes::Key::K_LSHIFT);
    } else {
      type(key);
    }
    i++;
  }
}

}  // namespace esp32_ps2dev
