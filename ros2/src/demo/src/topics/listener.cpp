#include <rclcpp/rclcpp.hpp>
#include <demo_msgs/msg/pod_large_image.hpp>
#include <demo_msgs/msg/pod_small_string.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener");

  char *large_var = std::getenv("LARGE");
  bool large = large_var && std::string(large_var) == "1";
  RCLCPP_INFO(node->get_logger(), "Large messages: %d", large);

  auto t0 = std::chrono::steady_clock::now();
  int count = 0;

  auto callback_small = [&](const demo_msgs::msg::PodSmallString &) {
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

  auto callback_large = [&](const demo_msgs::msg::PodLargeImage &) {
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

  std::shared_ptr<rclcpp::Subscription<demo_msgs::msg::PodSmallString>> sub_small;
  std::shared_ptr<rclcpp::Subscription<demo_msgs::msg::PodLargeImage>> sub_large;
  if (!large) {
    sub_small = node->create_subscription<demo_msgs::msg::PodSmallString>("chatter", rclcpp::SensorDataQoS(), callback_small);
  } else {
    sub_large = node->create_subscription<demo_msgs::msg::PodLargeImage>("chatter", rclcpp::SensorDataQoS(), callback_large);
  }
  rclcpp::spin(node);
  return 0;
}
