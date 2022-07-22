#ifndef __ESP32_PS2DEV_H__
#define __ESP32_PS2DEV_H__

#include "Arduino.h"

// Enable serial debug mode?
//#define _PS2DBG Serial

namespace esp32_ps2dev {

const uint32_t CLK_HALF_PERIOD_MICROS = 40;
const uint32_t CLK_QUATER_PERIOD_MICROS = CLK_HALF_PERIOD_MICROS / 2;
const uint32_t DEFAULT_WRITE_WAIT_MICROS = 1000;

class PS2dev {
 public:
  PS2dev(int clk, int data);

  enum class BusState {
    IDLE,
    COMMUNICATION_INHIBITED,
    HOST_REQUEST_TO_SEND,
  };

  int write(unsigned char data, uint32_t wait_ms = DEFAULT_WRITE_WAIT_MICROS);
  int write_multi(uint8_t len, uint8_t* data);
  int read(unsigned char* data, uint64_t timeout_ms = 30);
  int available();
  BusState get_bus_state();

 protected:
  int _ps2clk;
  int _ps2data;
  void golo(int pin);
  void gohi(int pin);
  void ack();
};

}  // namespace esp32_ps2dev

#endif  // __ESP32_PS2DEV_H__