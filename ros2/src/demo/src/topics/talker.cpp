#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <demo_msgs/msg/pod_large_image.hpp>
#include <demo_msgs/msg/pod_small_string.hpp>

using namespace std::chrono_literals;

namespace demo {

class Talker : public rclcpp::Node {
 public:
  explicit Talker(const rclcpp::NodeOptions &options) : Node("talker", options) {
    char *large_var = std::getenv("LARGE");
    large = large_var && std::string(large_var) == "1";
    RCLCPP_INFO(get_logger(), "Large messages: %d", large);

    if (!large) {
      pub_small = create_publisher<demo_msgs::msg::PodSmallString>("chatter", rclcpp::SensorDataQoS());
    } else {
      pub_large = create_publisher<demo_msgs::msg::PodLargeImage>("chatter", rclcpp::SensorDataQoS());
    }

    auto publish_message = [&]() {
      if (!large) {
        auto msg = pub_small->borrow_loaned_message();
        std::string text = "Hello World: " + std::to_string(count++);
        std::copy(text.begin(), text.end(), msg.get().message.data());
        pub_small->publish(std::move(msg));
      } else {
        auto msg = pub_large->borrow_loaned_message();
        memset(msg.get().image.data(), 42, 1000000);
        pub_large->publish(std::move(msg));
      }
    };

    auto on_timer = [&, publish_message=publish_message]() {
      while (rclcpp::ok()) {
        publish_message();
      }
    };

    timer = create_wall_timer(0s, on_timer);
  }

 private:
  bool large;
  size_t count = 0;
  std::shared_ptr<rclcpp::Publisher<demo_msgs::msg::PodSmallString>> pub_small;
  std::shared_ptr<rclcpp::Publisher<demo_msgs::msg::PodLargeImage>> pub_large;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace demo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Talker)
