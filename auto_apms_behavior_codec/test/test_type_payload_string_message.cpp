#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/util.hpp"

using auto_apms_behavior_codec::TypePayloadStringMessage;

// Minimal concrete type used to exercise the base-class protected helpers.
struct TestMsg : public TypePayloadStringMessage
{
  std::string type;
  std::string payload;

  TestMsg() = default;
  TestMsg(std::string t, std::string p) : type(std::move(t)), payload(std::move(p)) {}

  std::string encode() const override { return format(type, payload); }

  static TestMsg decode(const std::string & msg)
  {
    auto [t, p] = split(msg);
    return TestMsg{t, p};
  }
};

// ---------------------------------------------------------------------------
// format()
// ---------------------------------------------------------------------------

TEST(TypePayloadStringMessageTest, FormatProducesColonSeparated)
{
  TestMsg m{"FOO", "bar"};
  EXPECT_EQ(m.encode(), "FOO:bar");
}

TEST(TypePayloadStringMessageTest, FormatEmptyPayload)
{
  TestMsg m{"CMD", ""};
  EXPECT_EQ(m.encode(), "CMD:");
}

TEST(TypePayloadStringMessageTest, FormatEmptyType)
{
  TestMsg m{"", "payload"};
  EXPECT_EQ(m.encode(), ":payload");
}

TEST(TypePayloadStringMessageTest, FormatBothEmpty)
{
  TestMsg m{"", ""};
  EXPECT_EQ(m.encode(), ":");
}

// ---------------------------------------------------------------------------
// split()
// ---------------------------------------------------------------------------

TEST(TypePayloadStringMessageTest, SplitRoundTrip)
{
  TestMsg m = TestMsg::decode("HELLO:world");
  EXPECT_EQ(m.type, "HELLO");
  EXPECT_EQ(m.payload, "world");
}

TEST(TypePayloadStringMessageTest, SplitEmptyPayload)
{
  TestMsg m = TestMsg::decode("CMD:");
  EXPECT_EQ(m.type, "CMD");
  EXPECT_TRUE(m.payload.empty());
}

TEST(TypePayloadStringMessageTest, SplitMissingColonReturnsEmpty)
{
  TestMsg m = TestMsg::decode("NOCOLON");
  EXPECT_TRUE(m.type.empty());
  EXPECT_TRUE(m.payload.empty());
}

TEST(TypePayloadStringMessageTest, SplitEmptyString)
{
  TestMsg m = TestMsg::decode("");
  EXPECT_TRUE(m.type.empty());
  EXPECT_TRUE(m.payload.empty());
}

TEST(TypePayloadStringMessageTest, SplitOnlyColon)
{
  TestMsg m = TestMsg::decode(":");
  EXPECT_TRUE(m.type.empty());
  EXPECT_TRUE(m.payload.empty());
}

TEST(TypePayloadStringMessageTest, SplitPayloadContainingColonPreservesRemainder)
{
  // Only the first colon is the separator; everything after belongs to the payload.
  TestMsg m = TestMsg::decode("TYPE:a:b:c");
  EXPECT_EQ(m.type, "TYPE");
  EXPECT_EQ(m.payload, "a:b:c");
}

// ---------------------------------------------------------------------------
// encode/decode round-trip
// ---------------------------------------------------------------------------

TEST(TypePayloadStringMessageTest, RoundTripNonEmptyPayload)
{
  TestMsg original{"START", "MainTree(x=1)"};
  TestMsg recovered = TestMsg::decode(original.encode());
  EXPECT_EQ(recovered.type, original.type);
  EXPECT_EQ(recovered.payload, original.payload);
}

TEST(TypePayloadStringMessageTest, RoundTripEmptyPayload)
{
  TestMsg original{"CANCEL", ""};
  TestMsg recovered = TestMsg::decode(original.encode());
  EXPECT_EQ(recovered.type, original.type);
  EXPECT_TRUE(recovered.payload.empty());
}

