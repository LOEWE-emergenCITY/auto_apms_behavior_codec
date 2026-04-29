#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "auto_apms_behavior_codec/encoder_base_params.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_tree_message.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_codec
{

/**
 * @brief Common base class for behavior tree encoders.
 *
 * Manages the DictionaryManager and provides functionality to encode behavior-tree XML into the
 * compact CBOR format used by the codec layer. Derived classes (e.g. TreeEncoderSubscriber,
 * TreeEncoderExecutorProxy) decide *how* tree XML is received and what happens after encoding.
 *
 * ### Parameters (encoder_base_params)
 *
 * | Parameter         | Default           | Description                                            |
 * |-------------------|-------------------|--------------------------------------------------------|
 * | `node_manifest`   | `[]`              | String array of NodeManifest resource identities.      |
 * | `encoded_out_topic` | `encoded_tree`  | Topic to publish SerializedTreeMessage on.             |
 */
class BehaviorTreeEncoderBase : public rclcpp::Node
{
public:
  BehaviorTreeEncoderBase(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BehaviorTreeEncoderBase() override = default;

    /**
   * @brief Create a TreeDocument from XML and convert it to the internal representation.
   *
   * Node manifests known by the DictionaryManager are registered with the document so that
   * all node types can be resolved.
   */
  bool readTreeDefinitionFromXML(
    std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document> & document_out);

protected:
  /// Encode a behavior tree document into a CBOR byte vector.
  std::vector<uint8_t> encode(behavior_tree_representation::Document & document);

  /// Convert a TreeDocument into the internal representation used for encoding.
  bool readTreeDefinitionFromDocument(
    auto_apms_behavior_tree::core::TreeDocument & tree_doc,
    std::unique_ptr<behavior_tree_representation::Document> & document_out);



  /**
   * @brief Encode tree XML into CBOR bytes without publishing.
   *
   * @throws std::runtime_error if parsing or encoding fails.
   */
  std::vector<uint8_t> encodeToBytes(const std::string & tree_xml);

  /// Publish a pre-encoded CBOR byte vector as a SerializedTreeMessage.
  void publishEncoded(const std::vector<uint8_t> & encoded_data);

  /// Convenience method: encode tree XML and publish the result as a SerializedTreeMessage.
  bool encodeAndPublish(const std::string & tree_xml);

  /// @return The shared DictionaryManager instance.
  std::shared_ptr<DictionaryManager> getDictionaryManager() const;

  /// @return Merged NodeManifest built from the encoder's base parameters.
  auto_apms_behavior_tree::core::NodeManifest getNodeManifest() const;

private:
  /// One-time initialization called from the constructor (builds dictionary, creates publisher).
  void setupEncoder();

  /// Convert a single TreeDocument::NodeElement into the internal representation.
  behavior_tree_representation::Node getNodeFromElement(
    const auto_apms_behavior_tree::core::TreeDocument::NodeElement & node_element);

  encoder_base_params::ParamListener param_listener_;
  std::shared_ptr<DictionaryManager> dictionary_manager_;
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage>::SharedPtr encoded_publisher_;
};

}  // namespace auto_apms_behavior_codec
