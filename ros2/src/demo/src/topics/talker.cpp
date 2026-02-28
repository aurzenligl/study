#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <demo_msgs/msg/pod_large_image.hpp>
#include <demo_msgs/msg/pod_small_string.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");

  char *large_var = std::getenv("LARGE");
  bool large = large_var && std::string(large_var) == "1";
  RCLCPP_INFO(node->get_logger(), "Large messages: %d", large);

  size_t count = 1;
  std::shared_ptr<rclcpp::Publisher<demo_msgs::msg::PodSmallString>> pub_small;
  std::shared_ptr<rclcpp::Publisher<demo_msgs::msg::PodLargeImage>> pub_large;
  if (!large) {
    pub_small = node->create_publisher<demo_msgs::msg::PodSmallString>("chatter", rclcpp::SensorDataQoS());
  } else {
    pub_large = node->create_publisher<demo_msgs::msg::PodLargeImage>("chatter", rclcpp::SensorDataQoS());
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

  while (rclcpp::ok()) {
    publish_message();
  }

  // auto timer = node->create_wall_timer(1s, publish_message);
  // rclcpp::spin(node);

  return 0;
}
