#pragma once
#include <stdint.h>

int autoconnect_serial(const uint32_t &baud_rate, const char* port);
int serial_write(int serial_port, uint8_t *data, uint16_t &size);
int read_serial(int serial_port, uint8_t *readbuf, uint16_t &bufsize);
void close_serial(int serial_port);
