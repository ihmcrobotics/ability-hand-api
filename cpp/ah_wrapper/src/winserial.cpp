#include "winserial.h"
#include <stdio.h>

int connect_to_usb_serial(HANDLE *serial_handle, const char *com_port_name,
                          unsigned long baud) {
  /*First, connect to com port.
  TODO: add a method that scans  this and filters based on the device
  descriptor.
  */
  (*serial_handle) = CreateFileA(com_port_name, GENERIC_READ | GENERIC_WRITE, 0,
                                 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  DCB serial_params = {0};
  serial_params.DCBlength = sizeof(serial_params);
  serial_params.BaudRate = baud;
  serial_params.ByteSize = DATABITS_8;
  serial_params.StopBits = ONESTOPBIT;
  serial_params.Parity = PARITY_NONE;

  int rc = 0;
  rc |= SetCommState((*serial_handle), &serial_params);

  if (rc != 0) {
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts((*serial_handle), &timeouts);
  }
  return rc;
}

AHSerial::AHSerial(const uint32_t &baud_rate, const char* port) {
  char namestr[16] = {0};
  for (int i = 0; i < 255; i++) {
    int rl = sprintf_s(namestr, "\\\\.\\COM%d", i);
    int rc = connect_to_usb_serial(&serialport, namestr, baud_rate);
    if (rc != 0) {
      printf("Connected to COM port %s successfully\n", namestr);
      connected = true;
      return;
    }
  }
  printf("No COM ports found\n");
  return;
}

AHSerial::~AHSerial() {
  if (connected)
    CloseHandle(serialport);

  connected = false;
}

bool AHSerial::connected() const {
  return connected;
}

int AHSerial::serial_write(uint8_t *data, int size) const {
  if (!connected)
    return 0;

  LPDWORD written = 0;
  int wfrc = WriteFile(serialport, data, size, written, NULL);
  return (int)written;
}

int AHSerial::read_serial(uint8_t *readbuf, int bufsize) const {
  if (!connected)
    return 0;

  LPDWORD num_bytes_read = 0;
  int rc = ReadFile(serialport, readbuf, bufsize, (LPDWORD)(&num_bytes_read),
                    NULL); //
  return (int)num_bytes_read;
}
