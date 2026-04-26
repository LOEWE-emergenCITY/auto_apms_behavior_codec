#include <cctype>
#include <string>
#include <unordered_set>

#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/executor_feedback.hpp"

using auto_apms_behavior_codec::ExecutionState;
using auto_apms_behavior_codec::ExecutorFeedback;
using auto_apms_behavior_codec::ExecutorFeedbackType;
using auto_apms_behavior_codec::computeUniqueStringHash;

// ---------------------------------------------------------------------------
// computeBuildHash — output format
// ---------------------------------------------------------------------------

TEST(ComputeBuildHashTest, ProducesExactlyEightUppercaseHexChars)
{
  const std::string h = computeUniqueStringHash("some XML data");
  ASSERT_EQ(h.size(), 8u);
  for (char c : h) {
    EXPECT_TRUE(std::isxdigit(static_cast<unsigned char>(c))) << "non-hex char: " << c;
    // Output must be uppercase so the wire representation is canonical.
    EXPECT_FALSE(std::islower(static_cast<unsigned char>(c))) << "lowercase char: " << c;
  }
}

TEST(ComputeBuildHashTest, Deterministic)
{
  const std::string input = "<root MAIN_TREE_TO_EXECUTE=\"MainTree\"/>";
  EXPECT_EQ(computeUniqueStringHash(input), computeUniqueStringHash(input));
}

TEST(ComputeBuildHashTest, EmptyStringEqualsOffsetBasis)
{
  // FNV-1a 32-bit of the empty string equals the offset basis 2166136261 = 0x811C9DC5.
  EXPECT_EQ(computeUniqueStringHash(""), "811C9DC5");
}

TEST(ComputeBuildHashTest, SingleByteChangeAltersHash)
{
  // Even a one-character difference in tree XML must produce a different hash.
  EXPECT_NE(computeUniqueStringHash("TreeA"), computeUniqueStringHash("TreeB"));
  EXPECT_NE(computeUniqueStringHash("MainTree"), computeUniqueStringHash("MainTreo"));
}

TEST(ComputeBuildHashTest, WhitespaceChangeAltersHash)
{
  // Normalisation is the caller's responsibility; raw strings differing only in
  // whitespace must hash differently so the proxy can detect a real change.
  EXPECT_NE(computeUniqueStringHash("A B"), computeUniqueStringHash("AB"));
  EXPECT_NE(computeUniqueStringHash("tree\n"), computeUniqueStringHash("tree"));
}

TEST(ComputeBuildHashTest, PrefixesHashDifferently)
{
  // A common prefix must not cause two distinct trees to share a hash.
  EXPECT_NE(computeUniqueStringHash("MainTree"), computeUniqueStringHash("MainTree2"));
  EXPECT_NE(computeUniqueStringHash("Sub"), computeUniqueStringHash("SubTree"));
}

TEST(ComputeBuildHashTest, NoPracticalCollisionsForRealistInputs)
{
  // FNV-1a 32-bit has only 2^32 (~4 billion) possible output values, so collisions
  // are theoretically guaranteed for large enough input sets (birthday bound: ~65 k
  // inputs give a ~50 % collision probability).  For the small number of distinct
  // build requests expected in any real mission (tens to hundreds), the collision
  // risk is negligible — but this test makes that assumption explicit by checking
  // 1000 synthetic tree XMLs and expecting zero collisions.
  std::unordered_set<std::string> seen;
  seen.reserve(1000);
  for (int i = 0; i < 1000; ++i) {
    const std::string xml =
      "<root MAIN_TREE_TO_EXECUTE=\"Tree" + std::to_string(i) + "\"><BehaviorTree ID=\"Tree" +
      std::to_string(i) + "\"/></root>";
    const std::string h = computeUniqueStringHash(xml);
    EXPECT_TRUE(seen.insert(h).second)
      << "Hash collision detected for index " << i << " (hash=" << h << ")";
  }
}

// ---------------------------------------------------------------------------
// ExecutorFeedback — STATE type
// ---------------------------------------------------------------------------

TEST(ExecutorFeedbackStateTest, EncodeKnownStates)
{
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::IDLE).encode(), "STATE:0");
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::STARTING).encode(), "STATE:1");
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::RUNNING).encode(), "STATE:2");
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::PAUSED).encode(), "STATE:3");
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::HALTED).encode(), "STATE:4");
}

TEST(ExecutorFeedbackStateTest, DecodeKnownStates)
{
  EXPECT_EQ(ExecutorFeedback::decode("STATE:0").state, ExecutionState::IDLE);
  EXPECT_EQ(ExecutorFeedback::decode("STATE:1").state, ExecutionState::STARTING);
  EXPECT_EQ(ExecutorFeedback::decode("STATE:2").state, ExecutionState::RUNNING);
  EXPECT_EQ(ExecutorFeedback::decode("STATE:3").state, ExecutionState::PAUSED);
  EXPECT_EQ(ExecutorFeedback::decode("STATE:4").state, ExecutionState::HALTED);
}

TEST(ExecutorFeedbackStateTest, RoundTripAllStates)
{
  for (auto s : {ExecutionState::IDLE, ExecutionState::STARTING, ExecutionState::RUNNING,
                 ExecutionState::PAUSED, ExecutionState::HALTED})
  {
    const auto wire = ExecutorFeedback::makeState(s).encode();
    const auto f = ExecutorFeedback::decode(wire);
    EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
    EXPECT_EQ(f.state, s);
  }
}

TEST(ExecutorFeedbackStateTest, TypeFieldIsState)
{
  EXPECT_EQ(ExecutorFeedback::makeState(ExecutionState::RUNNING).type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(ExecutorFeedback::decode("STATE:2").type, ExecutorFeedbackType::STATE);
}

// ---------------------------------------------------------------------------
// ExecutorFeedback — REGISTERACK type
// ---------------------------------------------------------------------------

TEST(ExecutorFeedbackRegisterAckTest, EncodeContainsHashVerbatim)
{
  EXPECT_EQ(ExecutorFeedback::makeRegisterAck("A3F210BC").encode(), "REGISTERACK:A3F210BC");
}

TEST(ExecutorFeedbackRegisterAckTest, DecodeExtractsHash)
{
  const auto f = ExecutorFeedback::decode("REGISTERACK:A3F210BC");
  EXPECT_EQ(f.type, ExecutorFeedbackType::REGISTERACK);
  EXPECT_EQ(f.build_hash, "A3F210BC");
}

TEST(ExecutorFeedbackRegisterAckTest, RoundTripWithComputedHash)
{
  const std::string xml = "<root MAIN_TREE_TO_EXECUTE=\"M\"><BehaviorTree ID=\"M\"/></root>";
  const std::string hash = computeUniqueStringHash(xml);
  const auto f = ExecutorFeedback::decode(ExecutorFeedback::makeRegisterAck(hash).encode());
  EXPECT_EQ(f.type, ExecutorFeedbackType::REGISTERACK);
  EXPECT_EQ(f.build_hash, hash);
}

TEST(ExecutorFeedbackRegisterAckTest, TypeFieldIsRegisterAck)
{
  EXPECT_EQ(ExecutorFeedback::makeRegisterAck("00000000").type, ExecutorFeedbackType::REGISTERACK);
  EXPECT_EQ(ExecutorFeedback::decode("REGISTERACK:00000000").type, ExecutorFeedbackType::REGISTERACK);
}

// ---------------------------------------------------------------------------
// ExecutorFeedback — decode robustness
// ---------------------------------------------------------------------------

TEST(ExecutorFeedbackDecodeRobustnessTest, UnknownTypeYieldsDefaultState)
{
  const auto f = ExecutorFeedback::decode("BOGUS:payload");
  EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(f.state, ExecutionState::IDLE);
}

TEST(ExecutorFeedbackDecodeRobustnessTest, MissingColonYieldsDefaultState)
{
  const auto f = ExecutorFeedback::decode("NOCOLON");
  EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(f.state, ExecutionState::IDLE);
}

TEST(ExecutorFeedbackDecodeRobustnessTest, EmptyStringYieldsDefaultState)
{
  const auto f = ExecutorFeedback::decode("");
  EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(f.state, ExecutionState::IDLE);
}

TEST(ExecutorFeedbackDecodeRobustnessTest, NonIntegerStatePayloadYieldsIdle)
{
  const auto f = ExecutorFeedback::decode("STATE:notanint");
  EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(f.state, ExecutionState::IDLE);
}

TEST(ExecutorFeedbackDecodeRobustnessTest, EmptyStatePayloadYieldsIdle)
{
  const auto f = ExecutorFeedback::decode("STATE:");
  EXPECT_EQ(f.type, ExecutorFeedbackType::STATE);
  EXPECT_EQ(f.state, ExecutionState::IDLE);
}
