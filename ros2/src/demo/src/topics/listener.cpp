#include <rclcpp/rclcpp.hpp>
#include <demo_msgs/msg/pod_large_image.hpp>
#include <demo_msgs/msg/pod_small_string.hpp>

using namespace std::chrono_literals;

namespace demo {

class Listener : public rclcpp::Node {
 public:
  explicit Listener(const rclcpp::NodeOptions &options) : Node("listener", options) {
    char *large_var = std::getenv("LARGE");
    large = large_var && std::string(large_var) == "1";
    RCLCPP_INFO(get_logger(), "Large messages: %d", large);

    t0 = std::chrono::steady_clock::now();

    auto callback_small = [&](const demo_msgs::msg::PodSmallString &) {
      ++count;
      auto t1 = std::chrono::steady_clock::now();
      const std::chrono::duration<double> d1 = t1 - t0;
      if (d1.count() >= 1) {
        double freq = count / d1.count();
        t0 = t1;
        count = 0;
        RCLCPP_INFO(get_logger(), "Frequency: %.3f [Hz]", freq);
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
        RCLCPP_INFO(get_logger(), "Frequency: %.3f [Hz]", freq);
      }
    };

    if (!large) {
      sub_small = create_subscription<demo_msgs::msg::PodSmallString>("chatter", rclcpp::SensorDataQoS(), callback_small);
    } else {
      sub_large = create_subscription<demo_msgs::msg::PodLargeImage>("chatter", rclcpp::SensorDataQoS(), callback_large);
    }
  }

 private:
  bool large;
  int count = 0;
  std::chrono::steady_clock::time_point t0;
  std::shared_ptr<rclcpp::Subscription<demo_msgs::msg::PodSmallString>> sub_small;
  std::shared_ptr<rclcpp::Subscription<demo_msgs::msg::PodLargeImage>> sub_large;
};

}  // namespace demo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Listener)
