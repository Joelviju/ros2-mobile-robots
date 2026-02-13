#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <future>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ButlerTaskManager : public rclcpp::Node
{
public:
  ButlerTaskManager()
  : Node("butler_task_manager")
  {
    nav_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    nav_client_->wait_for_action_server();
    init_locations();
  }

  void run_orders(const std::vector<std::string> & tables)
  {
    canceled_tables_.insert("table2");

    if (!go_to("home")) return;
    wait_at_location();

    if (!go_to("kitchen")) return;
    wait_at_location();

    for (const auto & table : tables)
    {
      if (canceled_tables_.count(table))
        continue;

      current_target_ = table;

      if (!go_to(table))
        continue;

      if (!wait_for_confirmation(5s))
        continue;
    }

    go_to("kitchen");
    wait_at_location();
    go_to("home");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr active_goal_;

  std::map<std::string, geometry_msgs::msg::PoseStamped> locations_;
  std::set<std::string> canceled_tables_;

  bool task_canceled_{false};
  std::string current_target_;

  void init_locations()
  {
    locations_["home"] =
      make_pose(0.1406914319, 0.5081928539, 0.4983337561, 0.8669852753);

    locations_["kitchen"] =
      make_pose(-0.7337120293, 1.0928522301, 0.9551904994, 0.2959917395);

    locations_["table1"] =
      make_pose(-0.0364626681, 1.7256327010, 0.5775766731, 0.8163364421);

    locations_["table2"] =
      make_pose(0.9985553535, 2.1283621835, 0.6043914125, 0.7966875300);

    locations_["table3"] =
      make_pose(1.4900387697, 1.3027098000, -0.5384983007, 0.8426265959);
  }

  geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double qz, double qw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    return pose;
  }

  bool go_to(const std::string & location)
  {
    if (task_canceled_)
    {
      handle_cancel_recovery();
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose = locations_[location];

    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();

    auto options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this](GoalHandleNav::SharedPtr handle)
      {
        active_goal_ = handle;
      };

    options.result_callback =
      [promise](const GoalHandleNav::WrappedResult & result)
      {
        promise->set_value(
          result.code == rclcpp_action::ResultCode::SUCCEEDED);
      };

    nav_client_->async_send_goal(goal, options);
    rclcpp::spin_until_future_complete(shared_from_this(), future);

    return future.get();
  }

  void cancel_task()
  {
    task_canceled_ = true;
    if (active_goal_)
      nav_client_->async_cancel_goal(active_goal_);
  }

  void handle_cancel_recovery()
  {
    if (current_target_.find("table") != std::string::npos)
      go_to("kitchen");

    go_to("home");
  }

  void wait_at_location()
  {
    wait_non_blocking(5s);
  }

  void wait_non_blocking(std::chrono::seconds duration)
  {
    auto start = now();
    while ((now() - start) < rclcpp::Duration(duration))
    {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(50ms);
    }
  }

  bool wait_for_confirmation(std::chrono::seconds timeout)
  {
    auto start = now();
    while ((now() - start) < rclcpp::Duration(timeout))
    {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(100ms);
      if (simulate_confirmation())
        return true;
    }
    return false;
  }

  bool simulate_confirmation()
  {
    if (current_target_ == "table1")
      return false;
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ButlerTaskManager>();
  node->run_orders({"table1", "table2", "table3"});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
