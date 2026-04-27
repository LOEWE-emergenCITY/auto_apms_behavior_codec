#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"
#include "auto_apms_behavior_codec/behavior_tree_encoder_base.hpp"
#include "auto_apms_behavior_codec/tree_decoder_publisher.hpp"

TEST(BehaviorTreeEncodingDecodingWithAdditionalParametersTest, RoundTrip)
{
  // This test 
  // Example XML string for testing
  std::string example_xml = "<BehaviorTree ID=\"HelloWorldWithEntryPoint\"><Sequence><Logger message=\"Hello World!\" level=\"INFO\"/><Sleep msec=\"200\"/><Sequence><ForceSuccess><WasEntryUpdated entry=\"{name}\" _onSuccess=\"text := &apos;My name is &apos; + name\" _onFailure=\"text := &apos;My name is not given&apos;\"/></ForceSuccess><Logger message=\"{text}\" level=\"INFO\"/></Sequence><Sleep msec=\"200\"/></Sequence></BehaviorTree>";

  auto_apms_behavior_codec::BehaviorTreeEncoderBase encoder();
  auto_apms_behavior_codec::TreeDecoderPublisher decoder();
  std::cerr << "Original XML:\n" << example_xml << "\n\n";
  // testing still needs work, surprisingly complicated
}