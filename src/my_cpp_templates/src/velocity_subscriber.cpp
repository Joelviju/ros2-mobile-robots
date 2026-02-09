#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class VelocitySubscriber : public rclcpp::Node
{
public:
  VelocitySubscriber()
  : Node("velocity_subscriber")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      qos,
      std::bind(&VelocitySubscriber::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Velocity subscriber started");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double speed =
      std::sqrt(msg->linear.x * msg->linear.x +
                msg->linear.y * msg->linear.y);

    if (speed > 0.15)
    {
      RCLCPP_WARN(this->get_logger(),
                  "High speed detected: %.3f m/s", speed);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "Speed OK: %.3f m/s", speed);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySubscriber>());
  rclcpp::shutdown();
  return 0;
}
