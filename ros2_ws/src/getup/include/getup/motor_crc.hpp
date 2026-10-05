#pragma once

#include <cstdint>

#include "unitree_hg/msg/low_cmd.hpp"

namespace getup
{

// CRC32 used by Unitree (polynomial 0x04C11DB7, init 0xFFFFFFFF, no final
// xor), processed on 32-bit words MSB first.
uint32_t crc32_core(const uint32_t * ptr, uint32_t len);

// Computes and stores msg.crc over the packed unitree_hg LowCmd layout.
// Port of unitree_ros2/example/src/src/common/motor_crc_hg.cpp.
void set_crc(unitree_hg::msg::LowCmd & msg);

}  // namespace getup
