#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class QosSubscriber : public rclcpp::Node
{
public:
  QosSubscriber()
  : Node("qos_subscriber")
  {
    // MATCHING QoS (must match!)
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/sensor_data",
      qos,
      std::bind(&QosSubscriber::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "QoS subscriber started (best effort)");
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosSubscriber>());
  rclcpp::shutdown();
  return 0;
}
