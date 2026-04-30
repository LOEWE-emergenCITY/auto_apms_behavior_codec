#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <vector>
#include "telemetry_message_builder.hpp"

// Accepts "TelemetryEntryMessage" messages, accumulates them using the TelemetryMesssageBuilder. In case of duplicate key between to send messages, the newest value is used.
// Intervall between sending the accumulated message is defined by "send_interval_ms" node parameter. The input topic is defined by "input_topic" node parameter, and the output topic is defined by "output_topic" node parameter. The output message type is "SerializedTelemetryMessage".
class TelemetryAccumulator : public rclcpp::Node
{
public:
    TelemetryAccumulator(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~TelemetryAccumulator() override = default;
private:
    void telemetryCallback(const auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage::SharedPtr msg);
    telemetry_message_builder::TelemetryMessageBuilder telemetry_message_builder_;
}