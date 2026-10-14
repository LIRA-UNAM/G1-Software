#include <gtest/gtest.h>

#include <cmath>

#include "getup/observation.hpp"
#include "ppowalk/motion.hpp"

TEST(Tilt, FromQuaternion)
{
  const double h = std::sqrt(0.5);
  auto g = getup::projected_gravity(1, 0, 0, 0);
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 0.0, 1e-9);
  g = getup::projected_gravity(h, h, 0, 0);  // 90 deg roll: lying on the side
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 90.0, 1e-6);
  g = getup::projected_gravity(h, 0, h, 0);  // 90 deg pitch: lying on the back/front
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 90.0, 1e-6);
  const double a = 30.0 * M_PI / 180.0;  // 30 deg pitch
  g = getup::projected_gravity(std::cos(a / 2), 0, std::sin(a / 2), 0);
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 30.0, 1e-6);
  // Yaw does not tilt the robot.
  g = getup::projected_gravity(std::cos(0.6), 0, 0, std::sin(0.6));
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 0.0, 1e-6);
  g = getup::projected_gravity(0, 1, 0, 0);  // upside down
  EXPECT_NEAR(ppowalk::tilt_deg(g[0], g[1], g[2]), 180.0, 1e-6);
}

TEST(SafeMove, DurationRespectsPeakVelocity)
{
  const double T = ppowalk::safe_move_duration({0.0, 0.0}, {0.6, -0.3}, 0.3, 1.0);
  EXPECT_NEAR(T, 0.5 * M_PI * 0.6 / 0.3, 1e-9);
  EXPECT_DOUBLE_EQ(ppowalk::safe_move_duration({0.0}, {0.01}, 0.3, 2.0), 2.0);
  // Peak speed of the cosine profile equals max_vel.
  const double ds = 1e-6;
  const double v = 0.6 * (ppowalk::smooth_alpha(0.5 + ds) - ppowalk::smooth_alpha(0.5)) / (ds * T);
  EXPECT_NEAR(v, 0.3, 1e-4);
}

TEST(SafeMove, Blend)
{
  EXPECT_DOUBLE_EQ(ppowalk::smooth_alpha(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(ppowalk::smooth_alpha(2.0), 1.0);
  auto q = ppowalk::blend({0.0, 1.0}, {1.0, 3.0}, 0.5);
  EXPECT_DOUBLE_EQ(q[0], 0.5);
  EXPECT_DOUBLE_EQ(q[1], 2.0);
}

TEST(Remote, StickCommand)
{
  // Deadzone: exactly zero, so a released stick means "stand still".
  EXPECT_EQ(ppowalk::stick_command(0.0, 0.1, -0.7, 0.7), 0.0);
  EXPECT_EQ(ppowalk::stick_command(0.09, 0.1, -0.7, 0.7), 0.0);
  EXPECT_EQ(ppowalk::stick_command(-0.1, 0.1, -0.7, 0.7), 0.0);
  // Full deflection reaches the (asymmetric) limits.
  EXPECT_DOUBLE_EQ(ppowalk::stick_command(1.0, 0.1, -0.5, 0.7), 0.7);
  EXPECT_DOUBLE_EQ(ppowalk::stick_command(-1.0, 0.1, -0.5, 0.7), -0.5);
  // Linear between the deadzone and full deflection.
  EXPECT_NEAR(ppowalk::stick_command(0.55, 0.1, -0.7, 0.7), 0.35, 1e-12);
  EXPECT_NEAR(ppowalk::stick_command(-0.55, 0.1, -0.7, 0.7), -0.35, 1e-12);
  // Out of range / invalid input.
  EXPECT_DOUBLE_EQ(ppowalk::stick_command(3.0, 0.1, -0.7, 0.7), 0.7);
  EXPECT_EQ(ppowalk::stick_command(std::nan(""), 0.1, -0.7, 0.7), 0.0);
  // No deadzone; a zero limit blocks that direction.
  EXPECT_DOUBLE_EQ(ppowalk::stick_command(0.5, 0.0, -0.7, 0.7), 0.35);
  EXPECT_EQ(ppowalk::stick_command(-0.8, 0.1, 0.0, 0.7), 0.0);
}
