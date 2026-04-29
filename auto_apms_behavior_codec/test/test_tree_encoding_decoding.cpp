#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"
#include "auto_apms_behavior_codec/behavior_tree_encoder_base.hpp"
#include "auto_apms_behavior_codec/tree_decoder_publisher.hpp"

namespace
{

class TestBehaviorTreeEncoder : public auto_apms_behavior_codec::BehaviorTreeEncoderBase
{
public:
  explicit TestBehaviorTreeEncoder(const std::string & node_name)
  : auto_apms_behavior_codec::BehaviorTreeEncoderBase(node_name)
  {
  }

  using auto_apms_behavior_codec::BehaviorTreeEncoderBase::getDictionaryManager;
};

class RclcppTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

::testing::Environment * const rclcpp_test_environment =
  ::testing::AddGlobalTestEnvironment(new RclcppTestEnvironment());

std::string canonicalizeXml(const std::string & xml)
{
  tinyxml2::XMLDocument xml_document;
  const auto parse_result = xml_document.Parse(xml.c_str());
  EXPECT_EQ(parse_result, tinyxml2::XML_SUCCESS) << xml_document.ErrorStr();

  tinyxml2::XMLPrinter printer(nullptr, false);
  xml_document.Print(&printer);
  return std::string(printer.CStr());
}

std::filesystem::path examplesDir()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
         "auto_apms_behavior_codec/auto_apms_behavior_codec_examples/behavior";
}

std::string loadFile(const std::filesystem::path & file_path)
{
  std::ifstream input_stream(file_path);
  EXPECT_TRUE(input_stream.is_open()) << "Failed to open " << file_path.string();

  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  return buffer.str();
}

void expectRoundTripMatches(const std::filesystem::path & input_file, const std::string & expected_xml)
{
  const std::string example_xml = loadFile(input_file);
  ASSERT_FALSE(example_xml.empty());

  TestBehaviorTreeEncoder encoder("test_behavior_tree_encoder");
  auto_apms_behavior_codec::TreeDecoderPublisher decoder;

  std::unique_ptr<behavior_tree_representation::Document> document;
  ASSERT_TRUE(encoder.readTreeDefinitionFromXML(example_xml, document));
  ASSERT_NE(document, nullptr);

  const auto encoded_bytes = document->serialize(encoder.getDictionaryManager());
  ASSERT_FALSE(encoded_bytes.empty());

  behavior_tree_representation::Document decoded_document;
  ASSERT_TRUE(decoded_document.deserialize(encoded_bytes, encoder.getDictionaryManager()));

  const std::string reconstructed_xml = decoder.reconstructXML(decoded_document);
  ASSERT_FALSE(reconstructed_xml.empty());

  const std::string expected_canon = canonicalizeXml(expected_xml);
  const std::string reconstructed_canon = canonicalizeXml(reconstructed_xml);

  std::cerr << "--- Input XML: " << input_file << " ---\n" << canonicalizeXml(example_xml) << "\n";
  std::cerr << "--- Expected canonical XML ---\n" << expected_canon << "\n";
  std::cerr << "--- Reconstructed canonical XML ---\n" << reconstructed_canon << "\n";

  EXPECT_EQ(expected_canon, reconstructed_canon);
}

}  // namespace

TEST(BehaviorTreeEncodingDecodingExamplesTest, GotoExamples)
{
  expectRoundTripMatches(
    examplesDir() / "goto_examples.xml",
    R"xml(<root BTCPP_format="4" main_tree_to_execute="LandAtOrigin">
  <BehaviorTree ID="LandAtOrigin">
    <Sequence>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="drone0" x="0.0" y="0.0" z="0.0"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="PatrolOnce">
    <Sequence>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="{drone}" x="1.0" y="0.0" z="{z}"/>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="{drone}" x="1.0" y="1.0" z="{z}"/>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="{drone}" x="-1.0" y="1.0" z="{z}"/>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="{drone}" x="-1.0" y="0.0" z="{z}"/>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="{drone}" x="0.0" y="0.0" z="{z}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Test_GoTo">
    <Sequence>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="drone0" x="0.0" y="0.0" z="3.0"/>
      <SubTree _autoremap="false" ID="GoToAbsolute" drone="drone0" x="3.0" y="0.0" z="3.0"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="GoToAbsolute">
    <Sequence>
      <PublishGoToPosition frame_id="map" use_degrees="false" x="{z}" y="{y}" yaw="0.000000" z="{z}" drone="{drone}"/>
      <Sleep msec="2000" name="TODO"/>
    </Sequence>
  </BehaviorTree>
</root>)xml");
}

TEST(BehaviorTreeEncodingDecodingExamplesTest, HelloWorld)
{
  expectRoundTripMatches(
    examplesDir() / "hello_world.xml",
    R"xml(<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <SubTree _autoremap="false" ID="Print" msg="Hello"/>
      <Sleep msec="3000"/>
      <SubTree _autoremap="false" ID="Print" msg="World"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Print">
    <Logger level="INFO" message="{msg}"/>
  </BehaviorTree>
</root>)xml");
}

TEST(BehaviorTreeEncodingDecodingExamplesTest, HelloWorldWithConditions)
{
  expectRoundTripMatches(
    examplesDir() / "hello_world_with_conditions.xml",
    R"xml(<root BTCPP_format="4" main_tree_to_execute="HelloWorldWithEntryPoint">
  <BehaviorTree ID="HelloWorldWithEntryPoint">
    <Sequence>
      <Logger level="INFO" message="Hello World!"/>
      <Sleep msec="200"/>
      <Sequence>
        <ForceSuccess>
          <WasEntryUpdated entry="{name}" _onSuccess="text := &apos;My name is &apos; + name" _onFailure="text := &apos;My name is not given&apos;"/>
        </ForceSuccess>
        <Logger level="INFO" message="{text}"/>
      </Sequence>
      <Sleep msec="200"/>
    </Sequence>
  </BehaviorTree>
</root>)xml");
}