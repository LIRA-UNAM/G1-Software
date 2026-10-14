#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace getup
{

// Unitree wireless remote as packed in LowState.wireless_remote (40 bytes,
// layout of unitree_sdk2 example/g1/low_level/gamepad.hpp, little endian):
//   [0..1] head, [2..3] button bits, then floats lx, rx, ry, L2, ly.
// Sticks are in [-1, 1]: +lx / +rx = right, +ly / +ry = forward (up).
struct WirelessRemote
{
  // Bit order of the 16-bit button field (also the sensor_msgs/Joy button order).
  enum Button
  {
    kR1, kL1, kStart, kSelect, kR2, kL2, kF1, kF2,
    kA, kB, kX, kY, kUp, kRight, kDown, kLeft, kNumButtons
  };

  uint16_t keys{0};
  float lx{0.0f};
  float rx{0.0f};
  float ry{0.0f};
  float l2{0.0f};
  float ly{0.0f};

  bool pressed(Button b) const {return (keys >> b) & 1u;}
};

inline constexpr size_t kWirelessRemoteSize = 40;

inline const char * remote_button_name(int b)
{
  static const char * names[] = {"R1", "L1", "Start", "Select", "R2", "L2", "F1", "F2",
    "A", "B", "X", "Y", "Up", "Right", "Down", "Left"};
  return b >= 0 && b < WirelessRemote::kNumButtons ? names[b] : "?";
}

// Stick values that are not finite or far outside [-1, 1] (e.g. garbage while
// the remote links up) are reported as 0 rather than as full deflection.
inline float remote_axis(const uint8_t * data, size_t offset)
{
  float v;
  std::memcpy(&v, data + offset, sizeof(v));
  if (!std::isfinite(v) || std::abs(v) > 1.5f) {
    return 0.0f;
  }
  return std::clamp(v, -1.0f, 1.0f);
}

// data: the 40 bytes of LowState.wireless_remote.
inline WirelessRemote decode_wireless_remote(const uint8_t * data)
{
  WirelessRemote r;
  r.keys = static_cast<uint16_t>(data[2] | (data[3] << 8));
  r.lx = remote_axis(data, 4);
  r.rx = remote_axis(data, 8);
  r.ry = remote_axis(data, 12);
  r.l2 = remote_axis(data, 16);
  r.ly = remote_axis(data, 20);
  return r;
}

}  // namespace getup
