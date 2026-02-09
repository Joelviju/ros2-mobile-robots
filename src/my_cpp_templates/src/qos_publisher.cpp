#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class QosPublisher : public rclcpp::Node
{
public:
  QosPublisher()
  : Node("qos_publisher"), counter_(0)
  {
    // SENSOR DATA QoS (best effort, fast, lossy)
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/sensor_data", qos);

    timer_ = this->create_wall_timer(
      200ms,
      std::bind(&QosPublisher::publishData, this));

    RCLCPP_INFO(this->get_logger(), "QoS publisher started (best effort)");
  }

private:
  void publishData()
  {
    std_msgs::msg::String msg;
    msg.data = "Sensor reading #" + std::to_string(counter_++);

    publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Published: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosPublisher>());
  rclcpp::shutdown();
  return 0;
}
