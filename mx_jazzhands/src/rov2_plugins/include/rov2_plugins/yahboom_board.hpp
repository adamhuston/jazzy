#ifndef ROV2_PLUGINS__YAHBOOM_BOARD_HPP_
#define ROV2_PLUGINS__YAHBOOM_BOARD_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rov2_plugins
{

// Monotonic clock used for all telemetry freshness decisions. steady_clock is
// deliberate: a physical watchdog must not be fooled by a wall-clock jump.
using SteadyClock = std::chrono::steady_clock;

// Latest 9-axis IMU sample from the Yahboom onboard IMU (MPU9250 or ICM20948).
// Accel in m/s^2, gyro in rad/s, mag in raw sensor units.
struct ImuSample
{
  double ax {0.0}, ay {0.0}, az {0.0};
  double gx {0.0}, gy {0.0}, gz {0.0};
  double mx {0.0}, my {0.0}, mz {0.0};
  bool valid {false};
  uint64_t seq {0};                 // increments once per received frame
  SteadyClock::time_point stamp {};  // monotonic receive time
};

struct BatterySample
{
  double voltage {0.0};  // volts
  bool valid {false};
  uint64_t seq {0};
  SteadyClock::time_point stamp {};
};

struct EncoderSample
{
  std::array<int32_t, 4> counts {{0, 0, 0, 0}};  // m1..m4 cumulative ticks
  bool valid {false};
  uint64_t seq {0};
  SteadyClock::time_point stamp {};
};

// Structured connection + telemetry health. This is the evidence the Safety
// Czar / drivetrain reason about. It deliberately distinguishes four separate
// claims that must never be conflated:
//   open        - the serial device is open
//   responding  - the board has actually answered (firmware reply or fresh
//                 auto-report), not merely enumerated
//   *_age_ms    - how stale each telemetry stream is (-1 => never seen)
//   write_failures / last_error - transmit-side trouble
struct BoardHealth
{
  bool open {false};
  bool responding {false};
  double firmware_version {0.0};
  uint64_t write_failures {0};
  double imu_age_ms {-1.0};
  double battery_age_ms {-1.0};
  double encoder_age_ms {-1.0};
  std::string last_error;
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
  bool is_open() const {return fd_.load() >= 0;}

  // True only once the board has actually answered: a firmware-version reply was
  // received, or at least one telemetry stream is fresher than the telemetry
  // timeout. Opening the port alone does NOT make this true. Use this (not
  // is_open) to decide the board is a trustworthy actuation/telemetry peer.
  bool responding() const;

  // Structured connection + telemetry health snapshot (thread-safe).
  BoardHealth health() const;

  // Staleness window (ms) above which a telemetry stream is treated as not
  // fresh for responding()/health(). Defaults to 500 ms.
  void set_telemetry_timeout_ms(double ms) {telemetry_timeout_ms_.store(ms);}

  const std::string & device() const {return device_;}
  std::string last_error() const;

  // Command four wheel speeds, each in [-100, 100]. Values are clamped.
  // Thread-safe. Returns true only if the command frame was fully written to
  // the port; returns false (and records a write failure) if the port is closed
  // or the write did not complete. NOTE: a true result means TRANSMITTED, not
  // acknowledged or executed; the board does not ack motor frames.
  bool set_motor(int s1, int s2, int s3, int s4);

  // Latest telemetry snapshots (thread-safe copies). Each carries a monotonic
  // stamp and sequence number so callers can judge freshness themselves.
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
  bool write_frame(uint8_t func, const std::vector<uint8_t> & data);
  void enable_auto_report(bool enable);
  void request_version();

  void reader_loop();
  void parse_frame(uint8_t ext_type, const std::vector<uint8_t> & data);
  void set_error(const std::string & msg);
  void reset_samples();          // invalidate telemetry across (re)connect
  double age_ms(const SteadyClock::time_point & stamp, bool valid) const;

  std::string device_;
  std::atomic<int> fd_ {-1};

  std::thread reader_;
  std::atomic<bool> running_ {false};
  std::atomic<double> telemetry_timeout_ms_ {500.0};
  std::atomic<uint64_t> write_failures_ {0};

  // Serializes open()/close() transitions against each other.
  mutable std::mutex conn_mutex_;

  mutable std::mutex data_mutex_;
  ImuSample imu_;
  BatterySample battery_;
  EncoderSample encoders_;
  double version_ {0.0};
  uint64_t imu_seq_ {0};
  uint64_t battery_seq_ {0};
  uint64_t encoder_seq_ {0};

  mutable std::mutex write_mutex_;
  mutable std::mutex err_mutex_;
  std::string last_error_;
};

}  // namespace rov2_plugins

#endif  // ROV2_PLUGINS__YAHBOOM_BOARD_HPP_
