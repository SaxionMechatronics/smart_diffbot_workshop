#ifndef SMART_DIFFBOT_NAVIGATION__NAVIGATOR_PLUGINS__EXACT_PATH_NAVIGATOR_HPP_
#define SMART_DIFFBOT_NAVIGATION__NAVIGATOR_PLUGINS__EXACT_PATH_NAVIGATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "smart_diffbot_navigation/navigator_plugins/navigator.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace smart_diffbot_navigation
{

class ExactPathNavigator
  : public smart_diffbot_navigation::Navigator<nav2_msgs::action::NavigateThroughPoses>
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  typedef geometry_msgs::msg::PoseStamped Goal;

  ExactPathNavigator()
  : Navigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("navigate_exact_path");}

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:

  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace

#endif  
