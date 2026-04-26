#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/executor_command.hpp"

using auto_apms_behavior_codec::ExecutorCommand;
using auto_apms_behavior_codec::ExecutorCommandType;
using auto_apms_behavior_codec::executorCommandTypeToString;

TEST(ExecutorCommandTest, CommandTypeToString)
{
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::START), "START");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::STOP), "STOP");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::PAUSE), "PAUSE");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::RESUME), "RESUME");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::CANCEL), "CANCEL");
}

TEST(ExecutorCommandTest, EncodeWithPayload)
{
  auto cmd = ExecutorCommand::makeStart("my_tree").encode();
  EXPECT_EQ(cmd, "START:my_tree");
}

TEST(ExecutorCommandTest, EncodeEmptyPayload)
{
  auto cmd = ExecutorCommand::makeCancel().encode();
  EXPECT_EQ(cmd, "CANCEL:");
}

TEST(ExecutorCommandTest, EncodeComplexPayload)
{
  auto cmd = ExecutorCommand::makeStart("MyTree(param1=val1,param2=val2)").encode();
  EXPECT_EQ(cmd, "START:MyTree(param1=val1,param2=val2)");
}

TEST(ExecutorCommandTest, DecodeStart)
{
  auto cmd = ExecutorCommand::decode("START:MyTree");
  EXPECT_EQ(cmd.type, ExecutorCommandType::START);
  EXPECT_EQ(cmd.payload, "MyTree");
}

TEST(ExecutorCommandTest, DecodeCancel)
{
  auto cmd = ExecutorCommand::decode("CANCEL:");
  EXPECT_EQ(cmd.type, ExecutorCommandType::CANCEL);
  EXPECT_TRUE(cmd.payload.empty());
}

TEST(ExecutorCommandTest, DecodeUnknown)
{
  auto cmd = ExecutorCommand::decode("BOGUS:something");
  EXPECT_EQ(cmd.type, ExecutorCommandType::UNKNOWN);
}

TEST(ExecutorCommandTest, RoundTrip)
{
  const std::string wire = "START:MainTree(x=1)";
  auto cmd = ExecutorCommand::decode(wire);
  EXPECT_EQ(cmd.encode(), wire);
}
