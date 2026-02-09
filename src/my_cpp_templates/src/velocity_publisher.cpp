#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher()
  : Node("velocity_publisher"), tick_(0)
  {
    // Explicit QoS (important in ROS 2)
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&VelocityPublisher::publishVelocity, this));

    RCLCPP_INFO(this->get_logger(), "Velocity publisher started");
  }

private:
  void publishVelocity()
  {
    geometry_msgs::msg::Twist msg;

    // Generate smooth velocity pattern
    msg.linear.x = 0.2 * std::sin(0.1 * tick_);
    msg.linear.y = 0.0;
    msg.angular.z = 0.3;

    publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published linear.x=%.2f", msg.linear.x);

    tick_++;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tick_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}