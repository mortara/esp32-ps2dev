#ifndef BBF49036_AEBA_4E9F_A8ED_F5017C12A915
#define BBF49036_AEBA_4E9F_A8ED_F5017C12A915

#include "PS2Dev.hpp"
#include "ScanCodeSet2.h"

namespace esp32_ps2dev {

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
  void begin();
  bool data_reporting_enabled();
  bool is_scroll_lock_led_on();
  bool is_num_lock_led_on();
  bool is_caps_lock_led_on();
  void keydown(scancodes::Key key);
  void keyup(scancodes::Key key);
  void type(scancodes::Key key);
  void type(std::initializer_list<scancodes::Key> keys);
  void type(const char* str);

 protected:
  bool _data_reporting_enabled = true;
  bool _led_scroll_lock = false;
  bool _led_num_lock = false;
  bool _led_caps_lock = false;
};

}  // namespace esp32_ps2dev

#endif /* BBF49036_AEBA_4E9F_A8ED_F5017C12A915 */
