#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    auto qos = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      qos,
      std::bind(&ImageSubscriber::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Image subscriber started");
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Image received: %ux%u | encoding=%s | frame=%s",
      msg->width,
      msg->height,
      msg->encoding.c_str(),
      msg->header.frame_id.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
