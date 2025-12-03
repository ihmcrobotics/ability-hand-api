#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <unistd.h>    // write(), read(), close()

#include "linux_serial.h"

AHSerial::AHSerial(const uint32_t &baud_rate, const char *port) {
  char filename[32] = {0}; // some large enough empty buffer
  
  // If user declared port
  if (port) {
    strncpy(filename, port, sizeof(filename) - 1);
  }

  // If user declared filename
  if (filename[0]) {
    serial_port = open(filename, O_RDWR);
    if (serial_port < 0) {
      printf("Error %i from open %s: %s\n", errno, filename, strerror(errno));
      return;
    }
  } else {
    // Attempt to auto find serial port
    for (int i = 0; i < 256; i++) {
      sprintf(filename, "/dev/ttyUSB%d", i);
      serial_port = open(filename, O_RDWR);
      if (serial_port < 0) {
        printf("Error %i from open %s: %s\n", errno, filename, strerror(errno));
      } else {
        break;
      }
    }
  }

  // All attempts to connect to serial failed
  if (serial_port < 0) {
    printf("Exiting due to no serial port found\n");
    return;
  }

  // Setup serial connection
  struct termios2 tty;
  ioctl(serial_port, TCGETS2, &tty);
  tty.c_cflag &= ~CSIZE; // CSIZE is a mask for the number of bits per character
  tty.c_cflag |= CS8;    // 8 bits per byte (most common)
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // one stop bit
  tty.c_cflag &= ~CRTSCTS; // hw flow control off
  tty.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_oflag = 0;       // No remapping, no delays
  tty.c_oflag &= ~OPOST; // Make raw
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Turn off echo erase (echo erase only relevant if
                          // canonical input is active)
  tty.c_lflag &= ~ECHONL; //
  tty.c_lflag &= ~ISIG;   // Disables recognition of INTR (interrupt), QUIT
                          // and SUSP (suspend) characters

  // custom baud rate
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= CBAUDEX;
  // tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud_rate;
  tty.c_ospeed = baud_rate;

  // timeout=0
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  ioctl(serial_port, TCSETS2, &tty);

  printf("Connected to %s successfully\n", filename);
}

AHSerial::~AHSerial() {
  if (serial_port >= 0)
    close(serial_port);

  serial_port = -1;
}

bool AHSerial::connected() const {
  return serial_port >= 0;
}

int AHSerial::serial_write(uint8_t *data, uint16_t &size) const {
  if (serial_port < 0)
    return 0;

  return write(serial_port, data, size);
}

int AHSerial::read_serial(uint8_t *readbuf, uint16_t &bufsize) const {
  if (serial_port < 0)
    return 0;

  return read(serial_port, readbuf, bufsize);
}
