#pragma once

#include <chrono>

#include "hand.h"
#include "ppp.h"
#ifdef PLATFORM_WINDOWS
#include "winserial.h"
#elif defined(PLATFORM_LINUX)
#include "linux_serial.h"
#endif
#include <vector>

const uint16_t BUFFER_SIZE = 512;
const uint16_t STUFFED_BUFFER_SIZE = BUFFER_SIZE * 2;

enum Command { POSITION, VELOCITY, CURRENT, DUTY };

class AHWrapper {
public:
  AHWrapper(const uint8_t &hand_addr, const uint32_t &b_rate);
  ~AHWrapper();
  int connect();
  int read_write_once(const std::array<float, 6> &cmd_values,
                      const Command &cmd, const uint8_t &reply_mode);
  int write_once(const std::array<float, 6> &cmd_values,
      Command cmd,
      uint8_t reply_mode);
  bool read_once(uint8_t reply_mode);
  Hand hand;
  size_t n_reads = 0;
  size_t n_writes = 0;

private:
  std::array<uint8_t, BUFFER_SIZE> m_buffer;
  std::array<uint8_t, STUFFED_BUFFER_SIZE> m_stuffed_buffer;
  uint16_t m_buffer_idx;
  uint16_t m_stuffed_idx;
  const uint32_t baud_rate;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  static constexpr size_t MAX_STUFFED = STUFFED_BUFFER_SIZE;
  static constexpr size_t RX_BUF_SIZE = MAX_STUFFED*2;
  static constexpr uint32_t MAX_ATTEMPTS = 150000;
  Unstuffer unstuffer_;
  std::vector<uint8_t> stuffed_buf_;
  size_t bytes_read_;
  uint32_t attempts_;
  uint8_t rx_buf_[RX_BUF_SIZE];
};