#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener");

  auto t0 = std::chrono::steady_clock::now();
  int count = 0;

  auto callback = [&](const std_msgs::msg::String &) {
    ++count;
    auto t1 = std::chrono::steady_clock::now();
    const std::chrono::duration<double> d1 = t1 - t0;
    if (d1.count() >= 1) {
      double freq = count / d1.count();
      t0 = t1;
      count = 0;
      RCLCPP_INFO(node->get_logger(), "Frequency: %.3f [Hz]", freq);
    }
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub =
      node->create_subscription<std_msgs::msg::String>("chatter", rclcpp::SensorDataQoS(), callback);
  rclcpp::spin(node);
  return 0;
}
