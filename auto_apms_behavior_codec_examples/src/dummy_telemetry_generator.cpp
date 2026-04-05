#include <iomanip>

#include "auto_apms_behavior_codec/telemetry_message_builder.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_telemetry_message.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace auto_apms_behavior_codec;

class DummyTelemetryGenerator : public rclcpp::Node
{
public:
  DummyTelemetryGenerator() : Node("DummyTelemetryGenerator")
  {
    auto period_ms = 30000;
    publisher_ =
      this->create_publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage>("TelemetryOut", 10);
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&DummyTelemetryGenerator::onTimer, this));
    RCLCPP_INFO(this->get_logger(), "DummyTelemetryGenerator started with period %d ms", period_ms);
  }

private:
  void onTimer()
  {
    TelemetryMessageBuilder builder;
    // fixed telemetry values
    builder.addIntEntry("count", 42);
    builder.addFloatEntry("temperature", 21.5);
    builder.addBoolEntry("ok", true);
    builder.addStringEntry("info", "dummy");
    builder.addAnyTypeAllowedEntry("misc", "value", "std::string");

    auto serialized = builder.getSerializedMessage();
    auto msg = auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage();

    // Print serialized message as hex for debugging
    std::cout << "Telemetry serialized (hex, " << serialized.size() << " bytes): ";
    for (uint8_t byte : serialized) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(byte) & 0xff) << " ";
    }
    std::cout << std::dec << std::endl;

    msg.serialized_telemetry_message = serialized;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyTelemetryGenerator>());
  rclcpp::shutdown();
  return 0;
}
