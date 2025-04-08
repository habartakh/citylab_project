#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#define M_PI_6 0.5235983

using std::placeholders::_1;
using namespace std::chrono_literals;

// Number of laser rays : 720
// Laser scan ranges vector correspondances:
<<<<<<< HEAD
<<<<<<< HEAD
// 0-360 : BACK
// 165   : RIGHT
// 330   : FRONT
// 495   : LEFT
=======
// 0-720 : BACK
=======
// 0     : BACK
>>>>>>> 539d588 (Changed approach to compute safest direction to move robot)
// 180   : RIGHT
// 360   : FRONT
// 540   : LEFT
>>>>>>> 992f095 (Adapted the code to work on the real robot)

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_node") {

    // Use callback groups to improve shared ressources access
    scan_sub_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = scan_sub_callback_group_;

    // Create a subscriber to /scan topic
    laser_scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Patrol::scan_topic_callback, this, _1),
            options);

    // create a publisher to move the robot
    move_robot_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Control loop of 10Hz to publish cmd_vel messages
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group_);
    obstacle_detected = false;
  }

private:
  // Changed the approach to compute the safest direction to move the robot.
  // Instead of only working on a part of the laser_scan_msg.ranges[180:540],
  // we test the values of each angle to see if it is between[-pi/2,pi/2].
  // This is more robust since rays 180 and 540 do not correspond exactly to
  // -pi/2 and pi/2 because the scan topic rays do not start from angle -pi but
  // from -3.12414
  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    range_max = msg->range_max;             // maximum range value [m]
    angle_increment = msg->angle_increment; // angular distance b/w laser scan
                                            // measurements [rad]

    angle_min = msg->angle_min;
    angle_max = msg->angle_max;

    obstacle_detected = false;

    // std::cout << "range_max : " << range_max << std::endl;    // 12
    // std::cout << "angle_min : " << msg->angle_min << std::endl; // 3.12414
    // std::cout << "angle_max : " << msg->angle_max << std::endl; // 3.14159
    // std::cout<<"angle_increment:"<<angle_increment<<std::endl;// 0.00871451
    // std::cout << "ranges:" << msg->ranges.size() << std::endl; // 720

    int current_index = 0;
    int max_index = 0;
    float max_distance = 0.0; // Initialize to the smallest possible value

    for (auto it = msg->ranges.begin(); it != msg->ranges.end(); ++it) {

      float current_angle = angle_min + current_index * angle_increment;

      // we will only work with the 180 degrees laser rays in front of the robot
      // corresponding to [-pi/2,pi/2] rays
      if (current_angle >= -M_PI_2 && current_angle <= M_PI_2) {

        if (std::isfinite(*it) && *it > max_distance) {
          max_distance = *it;        // Update the maximum distance
          max_index = current_index; // Update the index the ray corresponding
                                     // to max distance
        }

        // obstacle detection in front of the robot between[-pi/6,pi/6]
        if (current_angle >= -M_PI_6 && current_angle <= M_PI_6) {
          if (std::isfinite(*it) && *it <= 0.35) {
            obstacle_detected = true;
          }
        }
      }
      current_index++;
    }
    // Calculate the corresponding angle of the largest distance
    direction_ = angle_min + max_index * angle_increment;
    std::cout << "direction_ : " << direction_ << std::endl;
  }

  // Move robot forward till detecting an obstacle less than 35cm
  // However, it is problematic to use only 1 ray to verify existence of
  // obstacles, instead we use all the rays drom [-pi/6,pi/6]
  void timer_callback() {
<<<<<<< HEAD
<<<<<<< HEAD
    this->twist_msg.linear.x = 0.1;
    if (obstacle_detected_forward()) {
=======
=======

    this->twist_msg.linear.x = 0.1;
    this->twist_msg.angular.z = 0.0;

>>>>>>> 539d588 (Changed approach to compute safest direction to move robot)
    if (obstacle_detected) {
>>>>>>> 992f095 (Adapted the code to work on the real robot)
      this->twist_msg.angular.z = direction_ / 2.0;
      RCLCPP_INFO(this->get_logger(), "Obstacle detected....");
    }

    move_robot_publisher->publish(this->twist_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  rclcpp::CallbackGroup::SharedPtr scan_sub_callback_group_;
  rclcpp::SubscriptionOptions options;

  float direction_; // safest direction robot can take to complete patrol
  float range_max;
  float angle_increment;
  float angle_min;
  float angle_max;
  bool obstacle_detected;
  std::vector<float> relevant_scan_ranges;

  geometry_msgs::msg::Twist twist_msg;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_robot_publisher;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();
  RCLCPP_INFO(patrol_node->get_logger(), "Patrol Node STARTING ...");

  // Use mutithreading for more efficiency
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}