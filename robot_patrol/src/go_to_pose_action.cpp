#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
// #include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <thread>

using namespace std::placeholders;
using GoToPoseMsg = robot_patrol::action::GoToPose;
using GoalHandleGoToPoseMsg = rclcpp_action::ServerGoalHandle<GoToPoseMsg>;
using Pose2D = geometry_msgs::msg::Pose2D;
// using Quaternion = geometry_msgs::msg::Quaternion;

class GoToPose : public rclcpp::Node {
public:
  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {

    this->action_server_ = rclcpp_action::create_server<GoToPoseMsg>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Starting Action Server...");

    // Create a subscriber to /scan topic
    odom_sub_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = odom_sub_callback_group_;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPose::odom_topic_callback, this, _1),
        sub_options);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseMsg>::SharedPtr action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::CallbackGroup::SharedPtr odom_sub_callback_group_;
  rclcpp::SubscriptionOptions sub_options;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  Pose2D desired_pos_;
  Pose2D current_pos_;

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Convert odometry orientation quaternion to euler angles from:
    // https://gist.github.com/simutisernestas/14047512cbffd355a5c29d0c4cbf0eb5
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = msg->pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, pitch{}, yaw{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, yaw);

    current_pos_.theta = yaw; // in rads
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // std::cout << "current_pos_.theta : " << current_pos_.theta << std::endl;
    // std::cout << "current_pos_.x : " << current_pos_.x << std::endl;
    // std::cout << "current_pos_.y : " << current_pos_.y << std::endl;
  }
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseMsg::Goal> goal) {
    // RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
    //             goal->secs);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPoseMsg> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleGoToPoseMsg> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPoseMsg> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    desired_pos_ = goal->goal_pos;
    auto feedback = std::make_shared<GoToPoseMsg::Feedback>();
    auto result = std::make_shared<GoToPoseMsg::Result>();

    rclcpp::Rate loop_rate(1);

    // Moving the robot to the goal position is done in two steps
    // First, turn the robot till it faces the goal
    // Second, avanced forward with constant linear speed till it reaches it
    auto move_to_goal = geometry_msgs::msg::Twist();
    float magnitude;
    float d_theta;

    // Make the robot face the goal
    d_theta = (desired_pos_.theta * M_PI / 180) - current_pos_.theta;
    while (std::abs(d_theta) > 0.22) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // desired_pos_.theta in degrees
      d_theta = (desired_pos_.theta * M_PI / 180) - current_pos_.theta;
      std::cout << "d_theta : " << d_theta << std::endl;

      move_to_goal.linear.x = 0.0;
      move_to_goal.angular.z = d_theta / 2;
      publisher_->publish(move_to_goal);

      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish rotation feedback");

      loop_rate.sleep();
    }

    // stop for 1 second
    RCLCPP_INFO(this->get_logger(), "Finished facing the right direction");
    move_to_goal.linear.x = 0.0;
    move_to_goal.angular.z = 0.0;
    publisher_->publish(move_to_goal);
    loop_rate.sleep();

    //  Then advance the robot in a straight line to the goal
    magnitude = sqrt(std::pow((desired_pos_.x - current_pos_.x), 2) +
                     std::pow((desired_pos_.y - current_pos_.y), 2));
    while (magnitude > 0.22) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      magnitude = sqrt(std::pow((desired_pos_.x - current_pos_.x), 2) +
                       std::pow((desired_pos_.y - current_pos_.y), 2));
      std::cout << "magnitude : " << magnitude << std::endl;

      move_to_goal.linear.x = 0.1;
      move_to_goal.angular.z = 0.0;
      publisher_->publish(move_to_goal);

      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish translation feedback");

      loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Finished moving forward");

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;

      move_to_goal.linear.x = 0.0;
      move_to_goal.angular.z = 0.0;
      publisher_->publish(move_to_goal);

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}