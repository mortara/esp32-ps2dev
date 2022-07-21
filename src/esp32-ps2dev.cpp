/*
 * ps2dev.cpp - an interface library for ps2 host.
 * limitations:
 *      we do not handle parity errors.
 *      The timing constants are hard coded from the spec. Data rate is
 *         not impressive.
 *      probably lots of room for optimization.
 */

#include "esp32-ps2dev.h"

namespace esp32_ps2dev {

/*
 * the clock and data pins can be wired directly to the clk and data pins
 * of the PS2 connector.  No external parts are needed.
 */
PS2dev::PS2dev(int clk, int data) {
  _ps2clk = clk;
  _ps2data = data;
  gohi(_ps2clk);
  gohi(_ps2data);
}

/*
 * according to some code I saw, these functions will
 * correctly set the clock and data pins for
 * various conditions.  It's done this way so you don't need
 * pullup resistors.
 */
void PS2dev::gohi(int pin) {
  digitalWrite(pin, HIGH);
  pinMode(pin, INPUT);
}

void PS2dev::golo(int pin) {
  pinMode(pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(pin, LOW);
}

int PS2dev::write(unsigned char data) {
#ifdef _PS2DBG
  _PS2DBG.print(F("sending "));
  _PS2DBG.println(data, HEX);
#endif

  // ensure the bus is idle for at least 50 microseconds
  for (size_t i = 0; i < 5; i++) {
    if (get_bus_state() != BusState::IDLE) {
      return -1;
    }
    delayMicroseconds(10);
  }

  // frame generation
  uint16_t frame = 0;
  uint8_t parity_bit = 1;
  for (size_t i = 0; i < 8; i++) {
    frame <<= 1;
    frame |= (data & 0x01);
    data >>= 1;
    parity_bit ^= frame & 0x01;
  }
  frame <<= 1;
  frame |= (parity_bit << 9);
  frame |= (0x01 << 10);

  // frame transmission
  for (size_t i = 0; i < 11; i++) {
    // stop writing if the clock is low
    if (digitalRead(_ps2clk) == LOW) {
      gohi(_ps2clk);
      gohi(_ps2data);
      return -2;
    }

    if (frame & 0x01) {
      gohi(_ps2data);
    } else {
      golo(_ps2data);
    }
    frame >>= 1;

    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
    golo(_ps2clk);
    delayMicroseconds(CLK_HALF_PERIOD_MICROS);
    gohi(_ps2clk);
    delayMicroseconds(CLK_QUATER_PERIOD_MICROS);
  }

#ifdef _PS2DBG
  _PS2DBG.print(F("sent "));
  _PS2DBG.println(data, HEX);
#endif

  return 0;
}

int PS2dev::write_multi(uint8_t len, uint8_t *data) {
  for (size_t i = 0; i < len; i++) {
    int ret = write(data[i]);
    if (ret != 0) {
      return -1;
    }
  }
  return 0;
}

int PS2dev::available() {
  // delayMicroseconds(BYTEWAIT);
  return ((digitalRead(_ps2data) == LOW) || (digitalRead(_ps2clk) == LOW));
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

int PS2dev::read(unsigned char *value, uint64_t timeout_ms) {
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

  *value = data & 0x00FF;

#ifdef _PS2DBG
  _PS2DBG.print(F("received data "));
  _PS2DBG.println(*value, HEX);
  _PS2DBG.print(F("received parity "));
  _PS2DBG.print(received_parity, BIN);
  _PS2DBG.print(F(" calculated parity "));
  _PS2DBG.println(received_parity, BIN);
#endif
  if (received_parity == calculated_parity) {
    return 0;
  } else {
    return -2;
  }
}

}  // namespace esp32_ps2dev