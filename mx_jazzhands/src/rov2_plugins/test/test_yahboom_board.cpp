#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "rov2_plugins/yahboom_board.hpp"

using rov2_plugins::BoardHealth;
using rov2_plugins::EncoderSample;
using rov2_plugins::YahboomBoard;

namespace
{

// Opens a pseudo-terminal master and returns its fd, writing the slave device
// path (which the board opens like a real serial port) into slave_path.
int open_master(std::string & slave_path)
{
  const int m = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (m < 0) {
    return -1;
  }
  if (::grantpt(m) != 0 || ::unlockpt(m) != 0) {
    ::close(m);
    return -1;
  }
  const char * sp = ::ptsname(m);
  if (sp == nullptr) {
    ::close(m);
    return -1;
  }
  slave_path = sp;
  return m;
}

// Builds an auto-report RX frame as the board expects it on the wire:
//   [0xFF, DEVICE_ID-1, LEN, TYPE, payload..., CHECK]
std::vector<uint8_t> build_rx(uint8_t type, const std::vector<uint8_t> & payload)
{
  const uint8_t ext_len = static_cast<uint8_t>(payload.size() + 3);
  uint8_t check = static_cast<uint8_t>(ext_len + type);
  for (uint8_t b : payload) {
    check = static_cast<uint8_t>(check + b);
  }
  std::vector<uint8_t> f = {0xFF, 0xFB, ext_len, type};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(check);
  return f;
}

void put_le32(std::vector<uint8_t> & v, int32_t x)
{
  const uint32_t u = static_cast<uint32_t>(x);
  v.push_back(static_cast<uint8_t>(u & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
}

// Reads whatever is available on fd within timeout_ms into out (appends).
void drain(int fd, std::vector<uint8_t> & out, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  uint8_t buf[256];
  while (std::chrono::steady_clock::now() < deadline) {
    struct pollfd pfd {fd, POLLIN, 0};
    const int r = ::poll(&pfd, 1, 20);
    if (r > 0 && (pfd.revents & POLLIN)) {
      const ssize_t n = ::read(fd, buf, sizeof(buf));
      for (ssize_t i = 0; i < n; ++i) {
        out.push_back(buf[i]);
      }
    }
  }
}

}  // namespace

TEST(YahboomBoard, OpenDoesNotImplyResponding)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());
  EXPECT_TRUE(board.is_open());
  // No telemetry has arrived yet: open != responding.
  EXPECT_FALSE(board.responding());

  board.close();
  ::close(master);
}

TEST(YahboomBoard, RespondsAfterVersionFrame)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());

  const auto frame = build_rx(0x51, {3, 3});  // version 3.3
  ASSERT_EQ(::write(master, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  bool responded = false;
  for (int i = 0; i < 50 && !responded; ++i) {
    responded = board.responding();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(responded);
  EXPECT_NEAR(board.firmware_version(), 3.3, 1e-6);

  board.close();
  ::close(master);
}

TEST(YahboomBoard, ParsesEncoderTelemetry)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());

  std::vector<uint8_t> payload;
  put_le32(payload, 100);
  put_le32(payload, -100);
  put_le32(payload, 200);
  put_le32(payload, -200);
  const auto frame = build_rx(0x0D, payload);  // FUNC_REPORT_ENCODER
  ASSERT_EQ(::write(master, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  EncoderSample s;
  for (int i = 0; i < 50 && !s.valid; ++i) {
    s = board.encoders();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(s.valid);
  EXPECT_EQ(s.counts[0], 100);
  EXPECT_EQ(s.counts[1], -100);
  EXPECT_EQ(s.counts[2], 200);
  EXPECT_EQ(s.counts[3], -200);

  const BoardHealth h = board.health();
  EXPECT_TRUE(h.open);
  EXPECT_GE(h.encoder_age_ms, 0.0);

  board.close();
  ::close(master);
}

TEST(YahboomBoard, RejectsBadChecksum)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());

  std::vector<uint8_t> payload;
  put_le32(payload, 11);
  put_le32(payload, 22);
  put_le32(payload, 33);
  put_le32(payload, 44);
  auto frame = build_rx(0x0D, payload);
  frame.back() ^= 0xFF;  // corrupt the checksum
  ASSERT_EQ(::write(master, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const EncoderSample s = board.encoders();
  EXPECT_FALSE(s.valid);  // corrupt frame must be dropped, not accepted

  board.close();
  ::close(master);
}

TEST(YahboomBoard, SetMotorTransmitsFramedCommand)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());

  std::vector<uint8_t> noise;
  drain(master, noise, 200);  // discard the open()-time auto-report/version frames

  ASSERT_TRUE(board.set_motor(10, 20, 30, 40));

  std::vector<uint8_t> rx;
  drain(master, rx, 300);

  // Find the motor frame: HEAD, DEVICE_ID(0xFC), LEN(7), FUNC_MOTOR(0x10), ...
  bool found = false;
  for (size_t i = 0; i + 8 < rx.size(); ++i) {
    if (rx[i] == 0xFF && rx[i + 1] == 0xFC && rx[i + 3] == 0x10) {
      EXPECT_EQ(rx[i + 4], 10u);
      EXPECT_EQ(rx[i + 5], 20u);
      EXPECT_EQ(rx[i + 6], 30u);
      EXPECT_EQ(rx[i + 7], 40u);
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);

  board.close();
  ::close(master);
}

TEST(YahboomBoard, WriteFailsWhenClosed)
{
  std::string slave;
  const int master = open_master(slave);
  ASSERT_GE(master, 0);

  YahboomBoard board(slave);
  ASSERT_TRUE(board.open());
  board.close();

  EXPECT_FALSE(board.is_open());
  EXPECT_FALSE(board.set_motor(0, 0, 0, 0));  // no silent success on a closed port
  EXPECT_FALSE(board.health().open);

  ::close(master);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
