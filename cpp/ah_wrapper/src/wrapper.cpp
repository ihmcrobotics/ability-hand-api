#include "wrapper.h"

#include <iostream>

#include "api.h"
#include "parser.h"
#include "serial_helper.h"

AHWrapper::AHWrapper(const uint8_t &hand_addr, const uint32_t &b_rate)
    : hand(hand_addr), baud_rate(b_rate), unstuffer(m_buffer.data(), BUFFER_SIZE) {}

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

int AHWrapper::connect(const char* port) {
  start_time = std::chrono::steady_clock::now();
  if (autoconnect_serial(baud_rate, port)) {
    return 1; // Could not connect
  } else {
    printf("Connected to Hand %d\n", hand.address);
    return 0;
  }
}

int AHWrapper::read(const uint8_t &reply_mode) {
  // Read some bytes
  uint16_t remaining_capacity = STUFFED_BUFFER_SIZE - total_bytes_read;
  int bytes_read = read_serial(m_stuffed_buffer.data() + total_bytes_read, remaining_capacity);

  // Loop through newly read bytes
  for (int i = total_bytes_read; i < total_bytes_read + bytes_read; ++i) {
    // Put each byte through the unstuffer
    uint16_t frame_length = unstuffer.unstuff_byte(m_stuffed_buffer[i]);

    // frame_length == 0 means we haven't received full frame yet
    if (frame_length == 0)
      continue;

    // Full frame received so we reset
    reset_read();

    // If checksum passed we parse the packet and return frame length
    if (compute_checksum(m_buffer.data(), frame_length)) {
      ++n_reads;
      parse_packet(m_buffer.data(), frame_length, hand, reply_mode);
      return frame_length;
    }

    // Checksum failed; return 0
    return 0;
  }

  total_bytes_read += bytes_read;
  return -1;
}

void AHWrapper::reset_read() {
  if (total_bytes_read != 0) {
    unstuffer = Unstuffer(m_buffer.data(), BUFFER_SIZE);
    total_bytes_read = 0;
  }
}

int AHWrapper::write(const std::array<float, 6> &cmd_values,
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

  reset_read();

  return m_stuffed_idx;
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

  return 0;
}
