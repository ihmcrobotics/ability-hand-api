#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdio.h>

#include "wrapper.h"
#include <thread>

int main(int argc, char *argv[]) {
  AHWrapper wrapper = AHWrapper(0x50, 1000000);
  wrapper.connect();

  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration<double>(now.time_since_epoch());
  std::array<float, 6> cmd = {30.0, 30.0, 30.0, 30.0, 30.0, -30};
  boolean write = false;
  float old = 0;

  for (size_t i = 0; i < 100000; i++) {

    // Calculate Hand Wave
    now = std::chrono::steady_clock::now();
    duration = std::chrono::duration<double>(now.time_since_epoch());
    for (size_t j = 0; j < cmd.size(); ++j) {
      double ft = static_cast<double>(duration.count()) * 3.0 +
          j * (2.0 * 3.14159265359 / 12.0);
      cmd[j] = (0.5 * std::sin(ft) + 0.5) * 45.0 + 15.0;
    }
    cmd[5] = -cmd[5];
    auto start = std::chrono::high_resolution_clock::now();
    if (!write)
    {
      wrapper.write_once(cmd, POSITION, 1);
      write = true;
    }
    auto finish = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    std::printf("write time = %llu \n", (unsigned long long)us);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    start = std::chrono::high_resolution_clock::now();
    int j = 0;
    boolean read = false;
    while(j < 30 && !read)
    {
      read = wrapper.read_once(1);
      ++j;
      write = false;
    }
    finish = std::chrono::high_resolution_clock::now();
    us = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    std::printf("read time = %llu \n", (unsigned long long)us);
    if (old != wrapper.hand.pos[0])
    {
      printf("%f %f %f %f %f %f\n", wrapper.hand.pos[0], wrapper.hand.pos[1], wrapper.hand.pos[2],
          wrapper.hand.pos[3], wrapper.hand.pos[4], wrapper.hand.pos[5]);
      old = wrapper.hand.pos[0];
    }
  }
}