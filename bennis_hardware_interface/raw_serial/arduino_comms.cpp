#include "bennis_hardware_interface/arduino_comms.hpp"
#include <stdexcept>
#include <cstring>
#include <vector>
#include <array>
#include <chrono>
#include <thread>

// POSIX 串列埠
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

namespace detail
{

  inline void pack_le(const Speeds &s, uint8_t out[8])
  {
    auto w = [&](int idx, int16_t v)
    {
      out[idx + 0] = static_cast<uint8_t>(v & 0xFF);
      out[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    w(0, s.fl);
    w(2, s.fr);
    w(4, s.rl);
    w(6, s.rr);
  }

  inline void unpack_le(const uint8_t in[8], Speeds &s)
  {
    auto r = [&](int idx) -> int16_t
    {
      return static_cast<int16_t>(static_cast<uint16_t>(in[idx]) | (static_cast<uint16_t>(in[idx + 1]) << 8));
    };
    s.fl = r(0);
    s.fr = r(2);
    s.rl = r(4);
    s.rr = r(6);
  }

  inline speed_t baud_to_termios(unsigned baud)
  {
    switch (baud)
    {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
    default:
      return B115200;
    }
  }

  // ---- 泛用 COBS 封包工具 --------------------------------------------
  // 將 payload 經 COBS 編碼並在尾端加 0x00，呼叫 write
  inline void write_packet_cobs(int fd, const uint8_t *payload, size_t len)
  {
    // COBS: 最壞情況輸出長度 = len + len/254 + 1
    const size_t enc_max = len + len / 254 + 1;
    std::vector<uint8_t> enc(enc_max + 1); // +1 放 delimiter 0x00

    cobs_encode_result er = cobs_encode(enc.data(), enc_max, payload, len);
    if (er.status != COBS_ENCODE_OK)
    {
      throw std::runtime_error("COBS encode failed");
    }
    enc[er.out_len] = 0x00; // delimiter
    const size_t out_len = er.out_len + 1;

    size_t left = out_len;
    const uint8_t *p = enc.data();
    while (left > 0)
    {
      ssize_t n = ::write(fd, p, left);
      if (n < 0)
        throw std::runtime_error("serial write failed");
      left -= static_cast<size_t>(n);
      p += static_cast<size_t>(n);
    }
  }

  // 從串列埠讀到下一個 0x00，COBS 解碼並回傳解碼後 payload
  inline bool read_packet_cobs(int fd,
                               std::vector<uint8_t> &decoded,
                               std::chrono::milliseconds timeout)
  {
    decoded.clear();
    std::vector<uint8_t> frame;
    auto deadline = std::chrono::steady_clock::now() + timeout;

    // 讀到 0x00 為止
    for (;;)
    {
      uint8_t b{};
      ssize_t n = ::read(fd, &b, 1);
      if (n == 1)
      {
        if (b == 0x00)
          break; // frame 結束
        frame.push_back(b);
        continue;
      }
      // 非阻塞 or 超時輪詢
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      ::usleep(1000);
    }

    if (frame.empty())
      return false;

    // 解碼：輸出緩衝需 >= 輸入長度
    decoded.resize(frame.size());
    cobs_decode_result dr = cobs_decode(decoded.data(), decoded.size(),
                                        frame.data(), frame.size());
    if (dr.status != COBS_DECODE_OK)
      return false;
    decoded.resize(dr.out_len);
    return true;
  }

} // namespace detail

// ---------------- ArduinoComms 實作 ----------------

ArduinoComms::ArduinoComms() = default;
ArduinoComms::~ArduinoComms() { close(); }

void ArduinoComms::open(const std::string &port, unsigned baud)
{
  std::scoped_lock lk(m_);
  close_nolock();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
    throw std::runtime_error("Failed to open " + port);

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0)
  {
    close_nolock();
    throw std::runtime_error("tcgetattr failed");
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS; // 無硬體流控
  tio.c_cflag &= ~PARENB;  // 8N1
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  auto b = detail::baud_to_termios(baud);
  cfsetispeed(&tio, b);
  cfsetospeed(&tio, b);

  // 非阻塞讀：靠我們自己的 timeout 輪詢
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    close_nolock();
    throw std::runtime_error("tcsetattr failed");
  }

  // 切回阻塞 I/O，避免 write 部分寫入
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  // Toggle DTR/RTS to reset typical Arduino boards, then wait for bootloader
  int mstat = 0;
  if (ioctl(fd_, TIOCMGET, &mstat) == 0)
  {
    mstat &= ~(TIOCM_DTR | TIOCM_RTS);
    ioctl(fd_, TIOCMSET, &mstat);
    ::usleep(100000); // 100 ms low
    mstat |= (TIOCM_DTR | TIOCM_RTS);
    ioctl(fd_, TIOCMSET, &mstat);
  }

  // Give the MCU time to reboot and print any boot noise, then flush RX
  std::this_thread::sleep_for(1s);
  tcflush(fd_, TCIFLUSH);

  port_ = port;
  baud_ = baud;
}

void ArduinoComms::close()
{
  std::scoped_lock lk(m_);
  close_nolock();
}

bool ArduinoComms::is_open() const
{
  std::scoped_lock lk(m_);
  return fd_ >= 0;
}

void ArduinoComms::ensure_open() const
{
  if (fd_ < 0)
    throw std::runtime_error("serial not open");
}

void ArduinoComms::close_nolock()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  port_.clear();
  baud_ = 0;
}

void ArduinoComms::flush_rx()
{
  std::scoped_lock lk(m_);
  if (fd_ >= 0)
    tcflush(fd_, TCIFLUSH);
}

// ---------------- 新協議 API ----------------------
// 傳 'm' + 8bytes，韌體回 8bytes（echo）
void ArduinoComms::send_motor_speeds(const Speeds &s)
{
  std::scoped_lock lk(m_);
  ensure_open();

  uint8_t buf[1 + 8];
  buf[0] = OP_SET_SPEEDS;
  detail::pack_le(s, &buf[1]);

  detail::write_packet_cobs(fd_, buf, sizeof(buf));
}

// 讀取剛才 'm' 的 echo（8 bytes -> Speeds）
std::optional<Speeds> ArduinoComms::read_echo(std::chrono::milliseconds timeout)
{
  std::scoped_lock lk(m_);
  ensure_open();

  std::vector<uint8_t> dec;
  if (!detail::read_packet_cobs(fd_, dec, timeout))
    return std::nullopt;
  if (dec.size() != 8)
    return std::nullopt;

  Speeds s{};
  detail::unpack_le(dec.data(), s);
  return s;
}



// 送 'v'，回 8 bytes（ticks/sec, int16）
std::optional<Speeds> ArduinoComms::read_speeds(std::chrono::milliseconds timeout)
{
  std::scoped_lock lk(m_);
  ensure_open();

  const uint8_t op = OP_READ_SPEEDS;
  detail::write_packet_cobs(fd_, &op, 1);

  std::vector<uint8_t> dec;
  if (!detail::read_packet_cobs(fd_, dec, timeout))
    return std::nullopt;
  if (dec.size() != 8)
    return std::nullopt;

  Speeds tps{};
  detail::unpack_le(dec.data(), tps);
  return tps;
}

// 送 'e'，回 16 bytes（4×int32 raw counts）
std::optional<std::array<int32_t, 4>>
ArduinoComms::read_encoders(std::chrono::milliseconds timeout)
{
  std::scoped_lock lk(m_);
  ensure_open();

  const uint8_t op = OP_READ_ENCODERS;
  detail::write_packet_cobs(fd_, &op, 1);

  std::vector<uint8_t> dec;
  if (!detail::read_packet_cobs(fd_, dec, timeout))
    return std::nullopt;
  if (dec.size() != 16)
    return std::nullopt;

  std::array<int32_t, 4> c{};
  auto rd = [&](int idx) -> int32_t
  {
    const uint8_t *p = dec.data() + idx;
    return static_cast<int32_t>(
        (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24));
  };
  c[0] = rd(0);
  c[1] = rd(4);
  c[2] = rd(8);
  c[3] = rd(12);
  return c;
}

// 保留一個「零速 ping」：送 m(0,0,0,0) 並等待 echo
bool ArduinoComms::ping(std::chrono::milliseconds timeout)
{
  Speeds zero{0, 0, 0, 0};
  send_motor_speeds(zero);
  return read_echo(timeout).has_value();
}
