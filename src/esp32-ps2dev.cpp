/*
 * ps2dev.cpp - an interface library for ps2 host.
 * limitations:
 *      we do not handle parity errors.
 *      The timing constants are hard coded from the spec. Data rate is
 *         not impressive.
 *      probably lots of room for optimization.
 */

#include "esp32-ps2dev.h"

//Enable serial debug mode?
//#define _PS2DBG Serial

//since for the device side we are going to be in charge of the clock,
//the two defines below are how long each _phase_ of the clock cycle is
#define CLKFULL 40
// we make changes in the middle of a phase, this how long from the
// start of phase to the when we drive the data line
#define CLKHALF 20

// Delay between bytes
// I've found i need at least 400us to get this working at all,
// but even more is needed for reliability, so i've put 1000us
#define BYTEWAIT 1000

// Timeout if computer not sending for 30ms
#define TIMEOUT 30

namespace esp32_ps2dev {

/*
 * the clock and data pins can be wired directly to the clk and data pins
 * of the PS2 connector.  No external parts are needed.
 */
PS2dev::PS2dev(int clk, int data)
{
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
void
PS2dev::gohi(int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void
PS2dev::golo(int pin)
{
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

int PS2dev::write(unsigned char data)
{
  delayMicroseconds(BYTEWAIT);

  unsigned char i;
  unsigned char parity = 1;

#ifdef _PS2DBG
  _PS2DBG.print(F("sending "));
  _PS2DBG.println(data,HEX);
#endif

  if (digitalRead(_ps2clk) == LOW) {
    return -1;
  }

  if (digitalRead(_ps2data) == LOW) {
    return -2;
  }

  golo(_ps2data);
  delayMicroseconds(CLKHALF);
  // device sends on falling clock
  golo(_ps2clk);	// start bit
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);

  for (i=0; i < 8; i++)
    {
      if (data & 0x01)
      {
        gohi(_ps2data);
      } else {
        golo(_ps2data);
      }
      delayMicroseconds(CLKHALF);
      golo(_ps2clk);
      delayMicroseconds(CLKFULL);
      gohi(_ps2clk);
      delayMicroseconds(CLKHALF);

      parity = parity ^ (data & 0x01);
      data = data >> 1;
    }
  // parity bit
  if (parity)
  {
    gohi(_ps2data);
  } else {
    golo(_ps2data);
  }
  delayMicroseconds(CLKHALF);
  golo(_ps2clk);
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);

  // stop bit
  gohi(_ps2data);
  delayMicroseconds(CLKHALF);
  golo(_ps2clk);
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);

  delayMicroseconds(BYTEWAIT);

#ifdef _PS2DBG
  _PS2DBG.print(F("sent "));
  _PS2DBG.println(data,HEX);
#endif

  return 0;
}

int PS2dev::write_multi(uint8_t len, uint8_t *data)
{
  for (size_t i = 0; i < len; i++) {
    int ret = write(data[i]);
    if (ret != 0) {
      return -1;
    }
  }
  return 0;
}

int PS2dev::available() {
  //delayMicroseconds(BYTEWAIT);
  return ( (digitalRead(_ps2data) == LOW) || (digitalRead(_ps2clk) == LOW) );
}

int PS2dev::read(unsigned char * value)
{
  unsigned int data = 0x00;
  unsigned int bit = 0x01;

  unsigned char calculated_parity = 1;
  unsigned char received_parity = 0;

  //wait for data line to go low and clock line to go high (or timeout)
  unsigned long waiting_since = millis();
  while((digitalRead(_ps2data) != LOW) || (digitalRead(_ps2clk) != HIGH)) {
    if((millis() - waiting_since) > TIMEOUT) return -1;
  }

  delayMicroseconds(CLKHALF);
  golo(_ps2clk);
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);

  while (bit < 0x0100) {
    if (digitalRead(_ps2data) == HIGH)
      {
        data = data | bit;
        calculated_parity = calculated_parity ^ 1;
      } else {
        calculated_parity = calculated_parity ^ 0;
      }

    bit = bit << 1;

    delayMicroseconds(CLKHALF);
    golo(_ps2clk);
    delayMicroseconds(CLKFULL);
    gohi(_ps2clk);
    delayMicroseconds(CLKHALF);

  }
  // we do the delay at the end of the loop, so at this point we have
  // already done the delay for the parity bit

  // parity bit
  if (digitalRead(_ps2data) == HIGH)
    {
      received_parity = 1;
    }

  // stop bit
  delayMicroseconds(CLKHALF);
  golo(_ps2clk);
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);


  delayMicroseconds(CLKHALF);
  golo(_ps2data);
  golo(_ps2clk);
  delayMicroseconds(CLKFULL);
  gohi(_ps2clk);
  delayMicroseconds(CLKHALF);
  gohi(_ps2data);


  *value = data & 0x00FF;

#ifdef _PS2DBG
  _PS2DBG.print(F("received data "));
  _PS2DBG.println(*value,HEX);
  _PS2DBG.print(F("received parity "));
  _PS2DBG.print(received_parity,BIN);
  _PS2DBG.print(F(" calculated parity "));
  _PS2DBG.println(received_parity,BIN);
#endif
  if (received_parity == calculated_parity) {
    return 0;
  } else {
    return -2;
  }

}

}