#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <mutex>
#include <chrono>

#include <cobs.h>


// 與韌體對齊的四馬達速度（int16，小端）
struct Speeds {
  int16_t fl, fr, rl, rr;
}; 

// ---- 新協議 opcode -------------------------------------------------
static constexpr uint8_t OP_SET_SPEEDS = 'm';  // 後接 4×int16，共 9 bytes
static constexpr uint8_t OP_READ_SPEEDS = 'v'; // 回 4×int16（ticks/sec）
static constexpr uint8_t OP_READ_ENCODERS = 'e'; // 回 4×int32（raw counts）


using namespace std::chrono_literals;

class ArduinoComms {
public:
  ArduinoComms();
  ~ArduinoComms();

  void open(const std::string& port, unsigned baud = 115200);
  void close();
  bool is_open() const;

  void send_motor_speeds(const Speeds& s);

  std::optional<Speeds> read_echo(std::chrono::milliseconds timeout = 0ms);
  std::optional<Speeds> read_speeds(std::chrono::milliseconds timeout = 100ms);
  std::optional<std::array<int32_t,4>> read_encoders(std::chrono::milliseconds timeout = 0ms);

  bool ping(std::chrono::milliseconds timeout = 100ms);

  void flush_rx();

  std::string port() const { return port_; }
  unsigned baud() const { return baud_; }

private:
  void ensure_open() const;
  void close_nolock();
  void write_all_nolock(const void* data, size_t len);
  bool read_frame_until_zero_nolock(std::vector<uint8_t>& frame,
                                    std::chrono::milliseconds timeout);

private:
  int fd_ {-1};
  std::string port_;
  unsigned baud_ {0};
  mutable std::mutex m_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H