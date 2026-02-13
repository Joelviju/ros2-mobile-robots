#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class Nav2Test : public rclcpp::Node {
public:
  Nav2Test() : Node("nav2_test") {
    client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&Nav2Test::send_goal, this));
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool sent_ = false;

  void send_goal() {
    if (sent_) return;

    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Nav2 not available");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.0;
    goal.pose.pose.orientation.w = 1.0;

    client_->async_send_goal(goal);
    sent_ = true;

    RCLCPP_INFO(get_logger(), "Goal sent");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Test>());
  rclcpp::shutdown();
}
