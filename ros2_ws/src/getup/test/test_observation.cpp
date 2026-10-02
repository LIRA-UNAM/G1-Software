#include <gtest/gtest.h>

#include <cmath>

#include "getup/observation.hpp"

namespace
{

void expect_vec3(const std::array<double, 3> & v, double x, double y, double z)
{
  EXPECT_NEAR(v[0], x, 1e-9);
  EXPECT_NEAR(v[1], y, 1e-9);
  EXPECT_NEAR(v[2], z, 1e-9);
}

}  // namespace

TEST(ProjectedGravity, Upright)
{
  expect_vec3(getup::projected_gravity(1, 0, 0, 0), 0, 0, -1);
}

TEST(ProjectedGravity, RollAndPitch)
{
  const double h = std::sqrt(0.5);
  // +90 deg roll (about x): gravity points along -y in the body frame.
  expect_vec3(getup::projected_gravity(h, h, 0, 0), 0, -1, 0);
  // +90 deg pitch (about y): gravity points along +x in the body frame.
  expect_vec3(getup::projected_gravity(h, 0, h, 0), 1, 0, 0);
  // Upside down (180 deg about x).
  expect_vec3(getup::projected_gravity(0, 1, 0, 0), 0, 0, 1);
}

TEST(ProjectedGravity, YawInvariant)
{
  const double a = 0.7;
  expect_vec3(getup::projected_gravity(std::cos(a / 2), 0, 0, std::sin(a / 2)), 0, 0, -1);
}

TEST(ObservationBuilder, LayoutMatchesTermOrder)
{
  const std::vector<double> defaults = {0.1, -0.2};
  getup::ObservationBuilder builder(
    {"base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions"}, defaults);
  ASSERT_EQ(builder.size(), 3u + 3u + 2u + 2u + 2u);

  getup::RobotState state;
  state.base_ang_vel = {1.0, 2.0, 3.0};
  state.base_quat = {1.0, 0.0, 0.0, 0.0};
  state.joint_pos = {0.5, 0.5};
  state.joint_vel = {-1.0, 4.0};
  const std::vector<float> last_action = {0.25f, -0.75f};

  std::vector<float> obs;
  builder.build(state, last_action, obs);
  const std::vector<float> expected = {
    1, 2, 3,           // base_ang_vel
    0, 0, -1,          // projected_gravity
    0.4f, 0.7f,        // joint_pos - default
    -1, 4,             // joint_vel
    0.25f, -0.75f};    // last action
  ASSERT_EQ(obs.size(), expected.size());
  for (size_t i = 0; i < obs.size(); ++i) {
    EXPECT_NEAR(obs[i], expected[i], 1e-6) << "index " << i;
  }
}

TEST(ObservationBuilder, ScalesAndOrder)
{
  getup::ObservationBuilder builder({"actions", "base_ang_vel"}, {0.0}, {2.0, 0.5});
  getup::RobotState state;
  state.base_ang_vel = {2.0, 4.0, 6.0};
  state.joint_pos = {0.0};
  state.joint_vel = {0.0};
  std::vector<float> obs;
  builder.build(state, {1.5f}, obs);
  const std::vector<float> expected = {3.0f, 1.0f, 2.0f, 3.0f};
  ASSERT_EQ(obs.size(), expected.size());
  for (size_t i = 0; i < obs.size(); ++i) {
    EXPECT_FLOAT_EQ(obs[i], expected[i]);
  }
}

TEST(ObservationBuilder, RejectsUnknownTerm)
{
  EXPECT_THROW(getup::ObservationBuilder({"base_lin_vel"}, {0.0}), std::invalid_argument);
}
