#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include "getup/wireless_remote.hpp"

namespace
{

void put(std::array<uint8_t, 40> & buf, size_t offset, float v)
{
  std::memcpy(buf.data() + offset, &v, sizeof(v));
}

}  // namespace

TEST(WirelessRemote, DecodesSdkLayout)
{
  std::array<uint8_t, 40> buf{};
  buf[0] = 0x55;  // head bytes are ignored
  buf[1] = 0x51;
  buf[2] = 0x04;  // bit 2: start
  buf[3] = 0x01;  // bit 8: A
  put(buf, 4, 0.25f);  // lx
  put(buf, 8, -0.5f);  // rx
  put(buf, 12, 0.75f);  // ry
  put(buf, 16, 1.0f);  // L2
  put(buf, 20, -1.0f);  // ly
  const auto r = getup::decode_wireless_remote(buf.data());
  EXPECT_EQ(r.keys, 0x0104);
  EXPECT_TRUE(r.pressed(getup::WirelessRemote::kStart));
  EXPECT_TRUE(r.pressed(getup::WirelessRemote::kA));
  EXPECT_FALSE(r.pressed(getup::WirelessRemote::kSelect));
  EXPECT_FALSE(r.pressed(getup::WirelessRemote::kR1));
  EXPECT_FLOAT_EQ(r.lx, 0.25f);
  EXPECT_FLOAT_EQ(r.rx, -0.5f);
  EXPECT_FLOAT_EQ(r.ry, 0.75f);
  EXPECT_FLOAT_EQ(r.l2, 1.0f);
  EXPECT_FLOAT_EQ(r.ly, -1.0f);
}

TEST(WirelessRemote, AllZeroIsIdle)
{
  std::array<uint8_t, 40> buf{};
  const auto r = getup::decode_wireless_remote(buf.data());
  EXPECT_EQ(r.keys, 0);
  EXPECT_EQ(r.lx, 0.0f);
  EXPECT_EQ(r.ly, 0.0f);
  EXPECT_EQ(r.rx, 0.0f);
}

TEST(WirelessRemote, GarbageSticksReadAsZero)
{
  std::array<uint8_t, 40> buf{};
  put(buf, 4, std::numeric_limits<float>::quiet_NaN());
  put(buf, 8, std::numeric_limits<float>::infinity());
  put(buf, 12, 1e30f);
  put(buf, 20, 1.2f);  // slightly past full deflection: clamped
  const auto r = getup::decode_wireless_remote(buf.data());
  EXPECT_EQ(r.lx, 0.0f);
  EXPECT_EQ(r.rx, 0.0f);
  EXPECT_EQ(r.ry, 0.0f);
  EXPECT_FLOAT_EQ(r.ly, 1.0f);
}
