#include <gtest/gtest.h>

#include "getup/motor_crc.hpp"

// Unitree's word-wise CRC equals CRC-32/MPEG-2 over the words' big-endian
// bytes. Reference values computed with a bytewise CRC-32/MPEG-2.
TEST(MotorCrc, MatchesCrc32Mpeg2)
{
  const uint32_t words[] = {0x31323334, 0x35363738};  // "12345678"
  EXPECT_EQ(getup::crc32_core(words, 2), 0x49e3c2fbu);
}

TEST(MotorCrc, ZeroLowCmd)
{
  // All-zero LowCmd: CRC covers the 1000 bytes before the crc field, so this
  // also checks the packed struct size and zeroed padding.
  unitree_hg::msg::LowCmd msg;
  getup::set_crc(msg);
  EXPECT_EQ(msg.crc, 0xfe172f9fu);
}

TEST(MotorCrc, ChangesWithContent)
{
  unitree_hg::msg::LowCmd a, b;
  b.motor_cmd[3].kp = 40.0f;
  getup::set_crc(a);
  getup::set_crc(b);
  EXPECT_NE(a.crc, b.crc);
}

TEST(MotorCrc, HeaderBytesAffectCrc)
{
  // Regression: mode_pr/mode_machine stores were optimized away when the
  // struct was read through a uint32_t pointer. Reference from a bytewise
  // CRC-32/MPEG-2 over the packed message.
  unitree_hg::msg::LowCmd msg;
  msg.mode_machine = 5;
  getup::set_crc(msg);
  EXPECT_EQ(msg.crc, 0x54f027edu);
}
