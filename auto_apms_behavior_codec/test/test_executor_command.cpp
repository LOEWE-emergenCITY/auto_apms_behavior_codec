#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/executor_command.hpp"

using auto_apms_behavior_codec::ExecutorCommandType;
using auto_apms_behavior_codec::executorCommandTypeToString;
using auto_apms_behavior_codec::formatExecutorCommand;

TEST(ExecutorCommandTest, CommandTypeToString)
{
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::START), "START");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::STOP), "STOP");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::PAUSE), "PAUSE");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::RESUME), "RESUME");
  EXPECT_EQ(executorCommandTypeToString(ExecutorCommandType::CANCEL), "CANCEL");
}

TEST(ExecutorCommandTest, FormatCommandWithPayload)
{
  auto cmd = formatExecutorCommand(ExecutorCommandType::START, "my_tree");
  EXPECT_EQ(cmd, "START:my_tree");
}

TEST(ExecutorCommandTest, FormatCommandEmptyPayload)
{
  auto cmd = formatExecutorCommand(ExecutorCommandType::CANCEL, "");
  EXPECT_EQ(cmd, "CANCEL:");
}

TEST(ExecutorCommandTest, FormatCommandComplexPayload)
{
  auto cmd = formatExecutorCommand(ExecutorCommandType::START, "MyTree(param1=val1,param2=val2)");
  EXPECT_EQ(cmd, "START:MyTree(param1=val1,param2=val2)");
}
