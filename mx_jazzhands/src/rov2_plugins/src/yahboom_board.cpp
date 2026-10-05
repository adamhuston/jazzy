#include "rov2_plugins/yahboom_board.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <map>
#include <thread>

namespace rov2_plugins
{

// --------------------------------------------------------------------------- //
// Shared-instance registry: one driver per device path, process-wide.
// --------------------------------------------------------------------------- //
std::shared_ptr<YahboomBoard> YahboomBoard::instance(const std::string & device)
{
  static std::mutex registry_mutex;
  static std::map<std::string, std::weak_ptr<YahboomBoard>> registry;

  std::lock_guard<std::mutex> lock(registry_mutex);
  auto it = registry.find(device);
  if (it != registry.end()) {
    if (auto existing = it->second.lock()) {
      return existing;
    }
  }
  auto board = std::make_shared<YahboomBoard>(device);
  registry[device] = board;
  return board;
}

YahboomBoard::YahboomBoard(std::string device)
: device_(std::move(device))
{
}

YahboomBoard::~YahboomBoard()
{
  if (is_open()) {
    set_motor(0, 0, 0, 0);  // fail-safe: never leave the wheels commanded
  }
  close();
}

// --------------------------------------------------------------------------- //
// Connection lifecycle
// --------------------------------------------------------------------------- //
bool YahboomBoard::open()
{
  if (is_open()) {
    return true;
  }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    set_error("open(" + device_ + ") failed: " + std::strerror(errno));
    return false;
  }
  if (!configure_port()) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  running_ = true;
  reader_ = std::thread(&YahboomBoard::reader_loop, this);

  // Let the port settle, then ask the board to stream telemetry and report
  // its firmware version. Neither command moves the robot.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  enable_auto_report(true);
  request_version();
  return true;
}

void YahboomBoard::close()
{
  running_ = false;
  if (reader_.joinable()) {
    reader_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool YahboomBoard::configure_port()
{
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    set_error(std::string("tcgetattr failed: ") + std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CRTSCTS;  // no hardware flow control

  // Non-canonical read with a 0.1 s timeout so the reader thread can poll the
  // running_ flag and exit promptly on close().
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    set_error(std::string("tcsetattr failed: ") + std::strerror(errno));
    return false;
  }
  tcflush(fd_, TCIFLUSH);
  return true;
}

// --------------------------------------------------------------------------- //
// Transmit framing  [HEAD, DEVICE_ID, LEN, FUNC, data..., CHECKSUM]
// --------------------------------------------------------------------------- //
void YahboomBoard::write_frame(uint8_t func, const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return;
  }
  std::vector<uint8_t> cmd;
  cmd.reserve(data.size() + 5);
  cmd.push_back(HEAD);
  cmd.push_back(DEVICE_ID);
  cmd.push_back(0);  // LEN placeholder
  cmd.push_back(func);
  cmd.insert(cmd.end(), data.begin(), data.end());
  cmd[2] = static_cast<uint8_t>(cmd.size() - 1);

  uint8_t checksum = COMPLEMENT;
  for (uint8_t b : cmd) {
    checksum = static_cast<uint8_t>(checksum + b);
  }
  cmd.push_back(checksum);

  std::lock_guard<std::mutex> lock(write_mutex_);
  ssize_t written = ::write(fd_, cmd.data(), cmd.size());
  if (written < 0) {
    set_error(std::string("write failed: ") + std::strerror(errno));
  }
}

void YahboomBoard::enable_auto_report(bool enable)
{
  const uint8_t state1 = enable ? 1 : 0;
  const uint8_t state2 = 0;  // not persisted to flash
  write_frame(FUNC_AUTO_REPORT, {state1, state2});
}

void YahboomBoard::request_version()
{
  write_frame(FUNC_REQUEST_DATA, {FUNC_VERSION, 0});
}

bool YahboomBoard::set_motor(int s1, int s2, int s3, int s4)
{
  if (fd_ < 0) {
    return false;
  }
  auto clamp8 = [](int v) -> uint8_t {
    v = std::max(-100, std::min(100, v));
    return static_cast<uint8_t>(static_cast<int8_t>(v));
  };
  write_frame(FUNC_MOTOR, {clamp8(s1), clamp8(s2), clamp8(s3), clamp8(s4)});
  return true;
}

// --------------------------------------------------------------------------- //
// Receive: byte state machine  [HEAD, DEVICE_ID-1, LEN, TYPE, data..., CHECK]
// --------------------------------------------------------------------------- //
void YahboomBoard::reader_loop()
{
  enum class State { Head1, Head2, Len, Type, Data };
  State state = State::Head1;
  uint8_t ext_len = 0;
  uint8_t ext_type = 0;
  uint8_t check_sum = 0;
  size_t data_len = 0;
  std::vector<uint8_t> ext_data;

  uint8_t buf[256];
  while (running_) {
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      if (n < 0 && errno != EAGAIN && errno != EINTR) {
        set_error(std::string("read failed: ") + std::strerror(errno));
      }
      continue;  // timeout (VTIME) or transient; re-check running_
    }
    for (ssize_t i = 0; i < n; ++i) {
      const uint8_t byte = buf[i];
      switch (state) {
        case State::Head1:
          if (byte == HEAD) {
            state = State::Head2;
          }
          break;
        case State::Head2:
          if (byte == static_cast<uint8_t>(DEVICE_ID - 1)) {
            state = State::Len;
          } else if (byte != HEAD) {
            state = State::Head1;
          }
          break;
        case State::Len:
          ext_len = byte;
          state = State::Type;
          break;
        case State::Type:
          ext_type = byte;
          check_sum = static_cast<uint8_t>(ext_len + ext_type);
          data_len = (ext_len >= 2) ? (ext_len - 2) : 0;
          ext_data.clear();
          if (data_len == 0) {
            state = State::Head1;  // malformed; resync
          } else {
            state = State::Data;
          }
          break;
        case State::Data:
          ext_data.push_back(byte);
          if (ext_data.size() == data_len) {
            const uint8_t rx_check = byte;  // last byte is the checksum
            if (check_sum == rx_check) {
              ext_data.pop_back();  // drop checksum; keep payload only
              parse_frame(ext_type, ext_data);
            }
            state = State::Head1;
          } else {
            check_sum = static_cast<uint8_t>(check_sum + byte);
          }
          break;
      }
    }
  }
}

namespace
{
inline int16_t rd_i16(const std::vector<uint8_t> & d, size_t o)
{
  return static_cast<int16_t>(d[o] | (d[o + 1] << 8));
}
inline int32_t rd_i32(const std::vector<uint8_t> & d, size_t o)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(d[o]) | (static_cast<uint32_t>(d[o + 1]) << 8) |
    (static_cast<uint32_t>(d[o + 2]) << 16) | (static_cast<uint32_t>(d[o + 3]) << 24));
}
}  // namespace

void YahboomBoard::parse_frame(uint8_t ext_type, const std::vector<uint8_t> & d)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  switch (ext_type) {
    case FUNC_REPORT_SPEED:
      if (d.size() >= 7) {
        battery_.voltage = static_cast<double>(d[6]) / 10.0;
        battery_.valid = true;
      }
      break;
    case FUNC_REPORT_MPU_RAW:
      if (d.size() >= 18) {
        const double gyro_ratio = 1.0 / 3754.9;   // +/-500 dps
        const double accel_ratio = 1.0 / 1671.84;  // +/-2 g
        imu_.gx = rd_i16(d, 0) * gyro_ratio;
        imu_.gy = rd_i16(d, 2) * -gyro_ratio;
        imu_.gz = rd_i16(d, 4) * -gyro_ratio;
        imu_.ax = rd_i16(d, 6) * accel_ratio;
        imu_.ay = rd_i16(d, 8) * accel_ratio;
        imu_.az = rd_i16(d, 10) * accel_ratio;
        imu_.mx = rd_i16(d, 12);
        imu_.my = rd_i16(d, 14);
        imu_.mz = rd_i16(d, 16);
        imu_.valid = true;
      }
      break;
    case FUNC_REPORT_ICM_RAW:
      if (d.size() >= 18) {
        const double ratio = 1.0 / 1000.0;
        imu_.gx = rd_i16(d, 0) * ratio;
        imu_.gy = rd_i16(d, 2) * ratio;
        imu_.gz = rd_i16(d, 4) * ratio;
        imu_.ax = rd_i16(d, 6) * ratio;
        imu_.ay = rd_i16(d, 8) * ratio;
        imu_.az = rd_i16(d, 10) * ratio;
        imu_.mx = rd_i16(d, 12) * ratio;
        imu_.my = rd_i16(d, 14) * ratio;
        imu_.mz = rd_i16(d, 16) * ratio;
        imu_.valid = true;
      }
      break;
    case FUNC_REPORT_ENCODER:
      if (d.size() >= 16) {
        encoders_.counts[0] = rd_i32(d, 0);
        encoders_.counts[1] = rd_i32(d, 4);
        encoders_.counts[2] = rd_i32(d, 8);
        encoders_.counts[3] = rd_i32(d, 12);
        encoders_.valid = true;
      }
      break;
    case FUNC_VERSION:
      if (d.size() >= 2) {
        version_ = static_cast<double>(d[0]) + static_cast<double>(d[1]) / 10.0;
      }
      break;
    default:
      break;
  }
}

// --------------------------------------------------------------------------- //
// Thread-safe accessors
// --------------------------------------------------------------------------- //
ImuSample YahboomBoard::imu() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return imu_;
}

BatterySample YahboomBoard::battery() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return battery_;
}

EncoderSample YahboomBoard::encoders() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return encoders_;
}

double YahboomBoard::firmware_version() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return version_;
}

void YahboomBoard::set_error(const std::string & msg)
{
  std::lock_guard<std::mutex> lock(err_mutex_);
  last_error_ = msg;
}

std::string YahboomBoard::last_error() const
{
  std::lock_guard<std::mutex> lock(err_mutex_);
  return last_error_;
}

}  // namespace rov2_plugins
