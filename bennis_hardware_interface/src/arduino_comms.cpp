// Boost.Asio + COBS-based implementation
#include "bennis_hardware_interface/arduino_comms.hpp"
#include <stdexcept>
#include <cstring>
#include <vector>
#include <array>
#include <chrono>
#include <cobs.h>

using namespace std::chrono_literals;

namespace detail
{
    inline void pack_le(const Speeds &s, uint8_t out[8])
    {
        auto write_word = [&](int idx, int16_t v)
        {
            out[idx + 0] = static_cast<uint8_t>(v & 0xFF);
            out[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        };
        write_word(0, s.fl);
        write_word(2, s.fr);
        write_word(4, s.rl);
        write_word(6, s.rr);
    }

    inline void unpack_le(const uint8_t in[8], Speeds &s)
    {
        auto read_word = [&](int idx) -> int16_t
        {
            return static_cast<int16_t>(static_cast<uint16_t>(in[idx]) |
                                        (static_cast<uint16_t>(in[idx + 1]) << 8));
        };
        s.fl = read_word(0);
        s.fr = read_word(2);
        s.rl = read_word(4);
        s.rr = read_word(6);
    }

    // Encode payload with COBS and append delimiter 0x00
    inline std::vector<uint8_t> cobs_encode_with_delim(const uint8_t *payload, size_t len)
    {
        const size_t enc_max = len + len / 254 + 1;
        std::vector<uint8_t> enc(enc_max + 1);
        cobs_encode_result er = cobs_encode(enc.data(), enc_max, payload, len);
        if (er.status != COBS_ENCODE_OK)
        {
            throw std::runtime_error("COBS encode failed");
        }
        enc[er.out_len] = 0x00;
        enc.resize(er.out_len + 1);
        return enc;
    }

    // Decode a COBS frame (no trailing 0x00 inside)
    inline bool cobs_decode_frame(const std::vector<uint8_t> &frame, std::vector<uint8_t> &decoded)
    {
        decoded.resize(frame.size());
        cobs_decode_result dr = cobs_decode(decoded.data(), decoded.size(), frame.data(), frame.size());
        if (dr.status != COBS_DECODE_OK)
        {
            return false;
        }
        decoded.resize(dr.out_len);
        return true;
    }
}

ArduinoComms::ArduinoComms() = default;
ArduinoComms::~ArduinoComms()
{
    close();
}

void ArduinoComms::open(const std::string &port, unsigned baud)
{
    std::scoped_lock lk(m_);
    close_nolock();
    boost::system::error_code ec;
    serial_ = std::make_unique<boost::asio::serial_port>(io_);
    serial_->open(port, ec);
    if (ec)
    {
        throw std::runtime_error("Failed to open serial port: " + ec.message());
    }
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    std::this_thread::sleep_for(std::chrono::seconds(2));

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
    return serial_ && serial_->is_open();
}

void ArduinoComms::ensure_open() const
{
    if (!serial_ || !serial_->is_open())
    {
        throw std::runtime_error("serial not open");
    }
}

void ArduinoComms::close_nolock()
{
    if (serial_ && serial_->is_open())
    {
        boost::system::error_code ec;
        serial_->cancel(ec);
        serial_->close(ec);
    }
    serial_.reset();

    port_.clear();
    baud_ = 0;
}

void ArduinoComms::flush_rx()
{
    std::scoped_lock lk(m_);
    if (!serial_ || !serial_->is_open()) return;
    // Drain any pending bytes without blocking
    std::array<uint8_t, 256> tmp{};
    boost::system::error_code ec;
    for (;;)
    {
        size_t n = serial_->read_some(boost::asio::buffer(tmp), ec);
        if (ec || n == 0) break;
    }
}

static bool read_cobs_frame_sync(boost::asio::serial_port &sp,
                                 std::vector<uint8_t> &frame,
                                 std::chrono::milliseconds /* timeout */)
{
    frame.clear();
    
    // Simple synchronous read until 0x00 delimiter
    boost::system::error_code ec;
    boost::asio::streambuf streambuf;
    
    // Read until we find 0x00 delimiter
    boost::asio::read_until(sp, streambuf, '\0', ec);
    
    if (ec)
    {
        return false;
    }
    
    // Extract data up to the delimiter
    const auto& data = streambuf.data();
    const uint8_t* ptr = static_cast<const uint8_t*>(data.data());
    size_t size = data.size();
    
    for (size_t i = 0; i < size; ++i) {
        if (ptr[i] == 0x00) {
            frame.assign(ptr, ptr + i);
            break;
        }
    }
    
    return !frame.empty();
}

void ArduinoComms::send_motor_speeds(const Speeds &s)
{
    std::scoped_lock lk(m_);
    ensure_open();

    uint8_t buf[1 + 8];
    buf[0] = OP_SET_SPEEDS;
    detail::pack_le(s, &buf[1]);

    auto enc = detail::cobs_encode_with_delim(buf, sizeof(buf));
    boost::asio::write(*serial_, boost::asio::buffer(enc));
}

std::optional<Speeds> ArduinoComms::read_echo(std::chrono::milliseconds timeout)
{
    std::scoped_lock lk(m_);
    ensure_open();

    std::vector<uint8_t> frame;
    if (!read_cobs_frame_sync(*serial_, frame, timeout))
    {
        return std::nullopt;
    }
    std::vector<uint8_t> dec;
    if (!detail::cobs_decode_frame(frame, dec) || dec.size() != 8)
    {
        return std::nullopt;
    }
    Speeds s{};
    detail::unpack_le(dec.data(), s);
    return s;
}

std::optional<Speeds> ArduinoComms::read_speeds(std::chrono::milliseconds timeout)
{
    std::scoped_lock lk(m_);
    ensure_open();

    const uint8_t op = OP_READ_SPEEDS;
    auto enc = detail::cobs_encode_with_delim(&op, 1);
    boost::asio::write(*serial_, boost::asio::buffer(enc));

    std::vector<uint8_t> frame;
    if (!read_cobs_frame_sync(*serial_, frame, timeout))
    {
        return std::nullopt;
    }
    std::vector<uint8_t> dec;
    if (!detail::cobs_decode_frame(frame, dec) || dec.size() != 8)
    {
        return std::nullopt;
    }
    Speeds tps{};
    detail::unpack_le(dec.data(), tps);
    return tps;
}

std::optional<std::array<int32_t, 4>> ArduinoComms::read_encoders(std::chrono::milliseconds timeout)
{
    std::scoped_lock lk(m_);
    ensure_open();

    const uint8_t op = OP_READ_ENCODERS;
    auto enc = detail::cobs_encode_with_delim(&op, 1);
    boost::asio::write(*serial_, boost::asio::buffer(enc));

    std::vector<uint8_t> frame;
    if (!read_cobs_frame_sync(*serial_, frame, timeout))
    {
        return std::nullopt;
    }
    std::vector<uint8_t> dec;
    if (!detail::cobs_decode_frame(frame, dec) || dec.size() != 16)
    {
        return std::nullopt;
    }

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

bool ArduinoComms::ping(std::chrono::milliseconds timeout)
{
    Speeds zero{0, 0, 0, 0};
    send_motor_speeds(zero);
    return read_echo(timeout).has_value();
}