#include "wrapper.h"

#include <iostream>

#include "api.h"
#include "parser.h"
#include "serial_helper.h"
uint16_t MAX_READ = 256;

AHWrapper::AHWrapper(const uint8_t &hand_addr, const uint32_t &b_rate)
    : hand(hand_addr), baud_rate(b_rate),
    unstuffer(m_buffer.data(), RX_BUF_SIZE),
    bytes_read(0) {}

AHWrapper::~AHWrapper() {
  printf("Closing connection to Hand %d\n", hand.address);
  std::chrono::duration<double> duration =
      std::chrono::steady_clock::now() - start_time;
  std::cout << "Rate: " << n_reads / duration.count() << std::endl;
  std::cout << "# of Reads: " << n_reads << "\n# Of Writes: " << n_writes
            << std::endl;
  std::cout << "Dropped Packet %: "
            << (1.0 -
                (static_cast<float>(n_reads) / static_cast<float>(n_writes))) *
                   100.0
            << std::endl;
  close_serial();
}

int AHWrapper::connect() {
  start_time = std::chrono::steady_clock::now();
  if (autoconnect_serial(baud_rate)) {
    return 1; // Could not connect
  } else {
    printf("Connected to Hand %d\n", hand.address);
    return 0;
  }
}

/* Takes in a array of command values, an enum command, and a reply mode.
    Writes the generated message to serial then reads bytes until it sees a
    complete byte stuffed frame using read_until() then parses received frame
    returned by read_until().
*/
int AHWrapper::read_write_once(const std::array<float, 6> &cmd_values,
                               const Command &cmd, const uint8_t &reply_mode) {
  switch (cmd) {
  case POSITION:
    m_buffer_idx =
        build_pos_msg(cmd_values, m_buffer, hand.address, reply_mode);
    break;
  case VELOCITY:
    m_buffer_idx =
        build_vel_msg(cmd_values, m_buffer, hand.address, reply_mode);
    break;
  case CURRENT:
    m_buffer_idx =
        build_tor_msg(cmd_values, m_buffer, hand.address, reply_mode);
    break;
  case DUTY:
    m_buffer_idx =
        build_dut_msg(cmd_values, m_buffer, hand.address, reply_mode);
    break;
  }

  m_stuffed_idx = ppp_stuff(m_buffer.data(), m_buffer_idx,
                            m_stuffed_buffer.data(), STUFFED_BUFFER_SIZE);
  serial_write(m_stuffed_buffer.data(), m_stuffed_idx);
  ++n_writes; // Can't determine if write fails or succeeds

  int unstuffed_bytes_read =
      read_until(m_stuffed_buffer.data(), m_buffer.data(), STUFFED_BUFFER_SIZE,
                 BUFFER_SIZE);
  if (unstuffed_bytes_read > 0) {
    // Response received, unstuffed and passed checksum
    ++n_reads;
    parse_packet(m_buffer.data(), unstuffed_bytes_read, hand, reply_mode);
    // printf("%f %f %f %f %f %f\n", hand.pos[0], hand.pos[1], hand.pos[2],
    //        hand.pos[3], hand.pos[4], hand.pos[5]);
  }
  bytes_read = 0;
  unstuffer = Unstuffer(m_buffer.data(), RX_BUF_SIZE);
  return 0;
}

int AHWrapper::write_once(const std::array<float, 6> &cmd_values,
      Command cmd,
      uint8_t reply_mode)
{
  switch (cmd) {
    case POSITION: m_buffer_idx = build_pos_msg(cmd_values, m_buffer, hand.address, reply_mode); break;
    case VELOCITY: m_buffer_idx = build_vel_msg(cmd_values, m_buffer, hand.address, reply_mode); break;
    case CURRENT:  m_buffer_idx = build_tor_msg(cmd_values, m_buffer, hand.address, reply_mode); break;
    case DUTY:     m_buffer_idx = build_dut_msg(cmd_values, m_buffer, hand.address, reply_mode); break;
  }
  m_stuffed_idx = ppp_stuff(m_buffer.data(), m_buffer_idx,
                            m_stuffed_buffer.data(), STUFFED_BUFFER_SIZE);
  serial_write(m_stuffed_buffer.data(), m_stuffed_idx);
  ++n_writes;
  return m_stuffed_idx;
}

bool AHWrapper::read_once(uint8_t reply_mode) {
  int result = read_serial(m_stuffed_buffer.data() + bytes_read, MAX_READ);
  if (result <= 0) {
    return false;
  }
  for (uint16_t idx = bytes_read; idx < bytes_read + result; ++idx) {
    uint16_t frame_len = unstuffer.unstuff_byte(m_stuffed_buffer[idx]);
    if (frame_len > 0) {
      if (compute_checksum(m_buffer.data(), frame_len)) {
        ++n_reads;
        parse_packet(m_buffer.data(), frame_len, hand, reply_mode);
      }
      else {
        std::printf("Checksum failed\n");
      }
      return true;
    }
  }
  bytes_read += result;
  return false;
}