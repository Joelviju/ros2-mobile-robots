//ros2 run my_cpp_templates basic_subscriber
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class BasicSubscriber : public rclcpp::Node
{
public:
  BasicSubscriber()
  : Node("basic_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter",
      10,
      std::bind(&BasicSubscriber::topicCallback, this, std::placeholders::_1));
  }

private:
  void topicCallback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(),
                "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
