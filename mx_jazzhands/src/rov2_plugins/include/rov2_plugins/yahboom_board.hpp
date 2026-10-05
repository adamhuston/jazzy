#ifndef ROV2_PLUGINS__YAHBOOM_BOARD_HPP_
#define ROV2_PLUGINS__YAHBOOM_BOARD_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rov2_plugins
{

// Latest 9-axis IMU sample from the Yahboom onboard IMU (MPU9250 or ICM20948).
// Accel in m/s^2, gyro in rad/s, mag in raw sensor units.
struct ImuSample
{
  double ax {0.0}, ay {0.0}, az {0.0};
  double gx {0.0}, gy {0.0}, gz {0.0};
  double mx {0.0}, my {0.0}, mz {0.0};
  bool valid {false};
};

struct BatterySample
{
  double voltage {0.0};  // volts
  bool valid {false};
};

struct EncoderSample
{
  std::array<int32_t, 4> counts {{0, 0, 0, 0}};  // m1..m4 cumulative ticks
  bool valid {false};
};

// Native C++ driver for the Yahboom YB-ERF01 / Rosmaster STM32 board, ported
// from Rosmaster_Lib V3.3.9. Owns ONE serial connection and a background reader
// thread that parses the board's auto-report telemetry (IMU, encoders, battery,
// speed). Thread-safe. The same physical board carries the drivetrain, the
// navigation IMU, and the battery, so a single shared instance is used by all
// three plugins (see instance()).
//
// It issues ONLY read-only telemetry requests plus motor commands; it never
// changes board configuration beyond enabling auto-report.
class YahboomBoard
{
public:
  // Shared, reference-counted instance keyed by device path. All plugins that
  // name the same device receive the same driver, so the single serial port is
  // never opened twice. Released automatically when the last holder drops it.
  static std::shared_ptr<YahboomBoard> instance(const std::string & device);

  explicit YahboomBoard(std::string device);
  ~YahboomBoard();

  YahboomBoard(const YahboomBoard &) = delete;
  YahboomBoard & operator=(const YahboomBoard &) = delete;

  // Open the serial port, start the reader thread, and enable auto-report.
  // Idempotent: subsequent calls while already open are no-ops and return true.
  bool open();
  void close();
  bool is_open() const { return fd_ >= 0; }

  const std::string & device() const { return device_; }
  std::string last_error() const;

  // Command four wheel speeds, each in [-100, 100]. Values are clamped.
  // Thread-safe; no-op (returns false) if the port is not open.
  bool set_motor(int s1, int s2, int s3, int s4);

  // Latest telemetry snapshots (thread-safe copies).
  ImuSample imu() const;
  BatterySample battery() const;
  EncoderSample encoders() const;
  double firmware_version() const;

private:
  // Wire protocol (Rosmaster V3.3.9).
  static constexpr uint8_t HEAD = 0xFF;
  static constexpr uint8_t DEVICE_ID = 0xFC;
  static constexpr uint8_t COMPLEMENT = 257 - DEVICE_ID;  // = 5
  static constexpr uint8_t FUNC_AUTO_REPORT = 0x01;
  static constexpr uint8_t FUNC_REPORT_SPEED = 0x0A;
  static constexpr uint8_t FUNC_REPORT_MPU_RAW = 0x0B;
  static constexpr uint8_t FUNC_REPORT_IMU_ATT = 0x0C;
  static constexpr uint8_t FUNC_REPORT_ENCODER = 0x0D;
  static constexpr uint8_t FUNC_REPORT_ICM_RAW = 0x0E;
  static constexpr uint8_t FUNC_MOTOR = 0x10;
  static constexpr uint8_t FUNC_REQUEST_DATA = 0x50;
  static constexpr uint8_t FUNC_VERSION = 0x51;

  bool configure_port();
  void write_frame(uint8_t func, const std::vector<uint8_t> & data);
  void enable_auto_report(bool enable);
  void request_version();

  void reader_loop();
  void parse_frame(uint8_t ext_type, const std::vector<uint8_t> & data);
  void set_error(const std::string & msg);

  std::string device_;
  int fd_ {-1};

  std::thread reader_;
  std::atomic<bool> running_ {false};

  mutable std::mutex data_mutex_;
  ImuSample imu_;
  BatterySample battery_;
  EncoderSample encoders_;
  double version_ {0.0};

  mutable std::mutex write_mutex_;
  mutable std::mutex err_mutex_;
  std::string last_error_;
};

}  // namespace rov2_plugins

#endif  // ROV2_PLUGINS__YAHBOOM_BOARD_HPP_
