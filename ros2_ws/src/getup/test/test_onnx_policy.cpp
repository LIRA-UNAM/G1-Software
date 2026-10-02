#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

#include "getup/onnx_policy.hpp"

// test/data/linear_93x29.onnx is y = W x + b with
//   W[i][j] = 0.01 * (((7 i + 3 j) % 11) - 5),  b[i] = 0.1 i
TEST(OnnxPolicy, LinearModel)
{
  getup::OnnxPolicy policy(std::string(GETUP_TEST_DATA_DIR) + "/linear_93x29.onnx");
  ASSERT_EQ(policy.num_obs(), 93u);
  ASSERT_EQ(policy.num_actions(), 29u);
  EXPECT_EQ(policy.input_name(), "obs");
  EXPECT_EQ(policy.output_name(), "actions");

  std::vector<float> obs(93);
  for (size_t j = 0; j < obs.size(); ++j) {
    obs[j] = 0.01f * static_cast<float>(j) - 0.3f;
  }
  // Run twice to check that buffers are reused correctly.
  for (int rep = 0; rep < 2; ++rep) {
    const auto & out = policy.infer(obs);
    for (int i = 0; i < 29; ++i) {
      double expected = 0.1 * i;
      for (int j = 0; j < 93; ++j) {
        expected += 0.01 * (((i * 7 + j * 3) % 11) - 5) * obs[j];
      }
      EXPECT_NEAR(out[i], expected, 1e-4) << "action " << i;
    }
  }
}

TEST(OnnxPolicy, RejectsWrongObsSize)
{
  getup::OnnxPolicy policy(std::string(GETUP_TEST_DATA_DIR) + "/linear_93x29.onnx");
  EXPECT_THROW(policy.infer(std::vector<float>(10)), std::invalid_argument);
}

// Optional: GETUP_POLICY_PATH=/path/to/g1_getup.onnx checks an exported
// getup policy has the expected 93 -> 29 interface and runs.
TEST(OnnxPolicy, ExportedGetupPolicy)
{
  const char * path = std::getenv("GETUP_POLICY_PATH");
  if (path == nullptr) {
    GTEST_SKIP() << "GETUP_POLICY_PATH not set";
  }
  getup::OnnxPolicy policy(path);
  ASSERT_EQ(policy.num_obs(), 93u);
  ASSERT_EQ(policy.num_actions(), 29u);
  const auto & out = policy.infer(std::vector<float>(93, 0.0f));
  ASSERT_EQ(out.size(), 29u);
}
