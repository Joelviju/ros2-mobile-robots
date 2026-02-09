#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class FakeCameraPublisher : public rclcpp::Node
{
public:
  FakeCameraPublisher()
  : Node("fake_camera_publisher"), frame_id_(0)
  {
    // Sensor data QoS (best effort, small queue)
    auto qos = rclcpp::SensorDataQoS();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos);

    timer_ = this->create_wall_timer(
      33ms,   // ~30 FPS
      std::bind(&FakeCameraPublisher::publishImage, this));

    RCLCPP_INFO(this->get_logger(), "Fake camera publisher started");
  }

private:
  void publishImage()
  {
    sensor_msgs::msg::Image img;

    // ---- Header ----
    img.header.stamp = this->now();
    img.header.frame_id = "camera_link";

    // ---- Image meta ----
    img.height = 480;
    img.width = 640;
    img.encoding = "rgb8";
    img.is_bigendian = false;
    img.step = img.width * 3; // 3 bytes per pixel (RGB)

    // ---- Fake image data ----
    img.data.resize(img.height * img.step);

    // Fill image with a simple pattern
    for (size_t i = 0; i < img.data.size(); i += 3)
    {
      img.data[i]     = frame_id_ % 255;  // R
      img.data[i + 1] = 100;              // G
      img.data[i + 2] = 200;              // B
    }

    publisher_->publish(img);
    frame_id_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
