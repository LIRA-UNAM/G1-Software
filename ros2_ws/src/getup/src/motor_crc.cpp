#include "getup/motor_crc.hpp"

#include <array>
#include <cstring>

namespace getup
{

namespace
{

// Memory layout expected by the robot firmware (natural alignment, 4-byte
// words): must not be reordered.
struct RawMotorCmd
{
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  uint32_t reserve;
};

struct RawLowCmd
{
  uint8_t mode_pr;
  uint8_t mode_machine;
  std::array<RawMotorCmd, 35> motor_cmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
};

static_assert(sizeof(RawMotorCmd) == 28, "unexpected MotorCmd layout");
static_assert(sizeof(RawLowCmd) == 1004, "unexpected LowCmd layout");

}  // namespace

uint32_t crc32_core(const uint32_t * ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  const uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    uint32_t xbit = 1u << 31;
    const uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc & 0x80000000) {
        crc <<= 1;
        crc ^= polynomial;
      } else {
        crc <<= 1;
      }
      if (data & xbit) {
        crc ^= polynomial;
      }
      xbit >>= 1;
    }
  }
  return crc;
}

void set_crc(unitree_hg::msg::LowCmd & msg)
{
  // Padding bytes are part of the checksum: zero them explicitly.
  RawLowCmd raw;
  std::memset(&raw, 0, sizeof(raw));
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;
  for (size_t i = 0; i < raw.motor_cmd.size(); i++) {
    const auto & m = msg.motor_cmd[i];
    raw.motor_cmd[i].mode = m.mode;
    raw.motor_cmd[i].q = m.q;
    raw.motor_cmd[i].dq = m.dq;
    raw.motor_cmd[i].tau = m.tau;
    raw.motor_cmd[i].kp = m.kp;
    raw.motor_cmd[i].kd = m.kd;
    raw.motor_cmd[i].reserve = m.reserve;
  }
  for (size_t i = 0; i < raw.reserve.size(); i++) {
    raw.reserve[i] = msg.reserve[i];
  }
  // Copy into words instead of casting the struct pointer: reading it through
  // a uint32_t* violates strict aliasing and lets the optimizer drop stores.
  std::array<uint32_t, sizeof(RawLowCmd) / 4> words;
  std::memcpy(words.data(), &raw, sizeof(raw));
  msg.crc = crc32_core(words.data(), static_cast<uint32_t>(words.size() - 1));
}

}  // namespace getup
