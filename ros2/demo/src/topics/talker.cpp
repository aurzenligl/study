#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");

  size_t count = 1;
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", rclcpp::SensorDataQoS());

  auto publish_message = [&]() {
    // auto msg = std::make_unique<std_msgs::msg::String>();
    auto msg = pub->borrow_loaned_message();
    msg.get().data = "Hello World: " + std::to_string(count++);
    pub->publish(std::move(msg));
  };

  while (rclcpp::ok()) {
    publish_message();
  }

  // auto timer = node->create_wall_timer(1s, publish_message);
  // rclcpp::spin(node);

  return 0;
}
