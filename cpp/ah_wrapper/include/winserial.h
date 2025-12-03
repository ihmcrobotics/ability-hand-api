#pragma once
#define NOMINMAX
#include <Windows.h>
#include <stdint.h>

class AHSerial {
public:
  // Connects to the specified port using the specified baud rate
  AHSerial(const uint32_t baud_rate, const char* port);
  // Disconnect from the serial port
  ~AHSerial();

  bool connected() const;
  int serial_write(uint8_t *data, uint16_t &size) const;
  int read_serial(uint8_t *readbuf, uint16_t &bufsize) const;

private:
  HANDLE serialport;
  bool connected = false;
}
