#ifndef __ESP32_PS2DEV_H__
#define __ESP32_PS2DEV_H__

#include "Arduino.h"

namespace esp32_ps2dev {
class PS2dev {
 public:
  PS2dev(int clk, int data);

  int write(unsigned char data);
	int write_multi(uint8_t len, uint8_t *data);
  int read(unsigned char* data);
  int available();

 protected:
  int _ps2clk;
  int _ps2data;
  void golo(int pin);
  void gohi(int pin);
  void ack();
};
}  // namespace esp32_ps2dev

#endif  // __ESP32_PS2DEV_H__