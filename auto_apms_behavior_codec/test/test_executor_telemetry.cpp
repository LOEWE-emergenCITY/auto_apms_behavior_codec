#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/executor_telemetry.hpp"

using auto_apms_behavior_codec::ExecutionState;
using auto_apms_behavior_codec::ExecutorTelemetry;

TEST(ExecutorTelemetryTest, EncodeDecodeRoundtrip)
{
  ExecutorTelemetry original;
  original.state = ExecutionState::RUNNING;
  original.behaviors = {"tree_a", "tree_b", "tree_c"};

  auto encoded = original.encode();
  ASSERT_FALSE(encoded.empty());

  auto decoded = ExecutorTelemetry::decode(encoded);
  EXPECT_EQ(decoded.state, ExecutionState::RUNNING);
  ASSERT_EQ(decoded.behaviors.size(), 3u);
  EXPECT_EQ(decoded.behaviors[0], "tree_a");
  EXPECT_EQ(decoded.behaviors[1], "tree_b");
  EXPECT_EQ(decoded.behaviors[2], "tree_c");
}

TEST(ExecutorTelemetryTest, EncodeDecodeAllStates)
{
  for (auto state :
       {ExecutionState::IDLE, ExecutionState::STARTING, ExecutionState::RUNNING, ExecutionState::PAUSED,
        ExecutionState::HALTED})
  {
    ExecutorTelemetry original;
    original.state = state;
    auto decoded = ExecutorTelemetry::decode(original.encode());
    EXPECT_EQ(decoded.state, state);
  }
}

TEST(ExecutorTelemetryTest, EncodeDecodeEmptyBehaviors)
{
  ExecutorTelemetry original;
  original.state = ExecutionState::IDLE;
  original.behaviors = {};

  auto decoded = ExecutorTelemetry::decode(original.encode());
  EXPECT_EQ(decoded.state, ExecutionState::IDLE);
  EXPECT_TRUE(decoded.behaviors.empty());
}

TEST(ExecutorTelemetryTest, DecodeEmptyData)
{
  auto decoded = ExecutorTelemetry::decode({});
  EXPECT_EQ(decoded.state, ExecutionState::IDLE);
  EXPECT_TRUE(decoded.behaviors.empty());
}

TEST(ExecutorTelemetryTest, DecodeInvalidData)
{
  std::vector<uint8_t> garbage = {0xFF, 0xFE, 0x00, 0x01};
  auto decoded = ExecutorTelemetry::decode(garbage);
  EXPECT_EQ(decoded.state, ExecutionState::IDLE);
  EXPECT_TRUE(decoded.behaviors.empty());
}

TEST(ExecutorTelemetryTest, DefaultState)
{
  ExecutorTelemetry telemetry;
  EXPECT_EQ(telemetry.state, ExecutionState::IDLE);
  EXPECT_TRUE(telemetry.behaviors.empty());
}
