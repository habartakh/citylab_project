#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <limits>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

// Number of laser rays : 720
// Laser scan ranges vector correspondances:
<<<<<<< HEAD
// 0-360 : BACK
// 165   : RIGHT
// 330   : FRONT
// 495   : LEFT
=======
// 0-720 : BACK
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

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group_);
    obstacle_detected = false;
  }

private:
  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    range_max = msg->range_max;             // maximum range value [m]
    angle_increment = msg->angle_increment; // angular distance b/w laser scan
                                            // measurements [rad]

    // we will only work with the 180 degrees laser rays in front of the robot
    // thus, relevant_scan_ranges[180:540] corresponding to [-pi/2,pi/2] rays
    relevant_scan_ranges = std::vector<float>(msg->ranges.begin() + 180,
                                              msg->ranges.begin() + 540);

    get_safest_direction();
    obstacle_detected = obstacle_detected_forward();
  }

  // Get the ray corresponding to maximal distance (other than inf) around
  // the robot
  int get_max_distance_index() {
    int max_index = 0;
    double max_distance = 0.0; // Initialize to the smallest possible value

    for (auto it = relevant_scan_ranges.begin();
         it != relevant_scan_ranges.end(); ++it) {
      if (*it<range_max && * it> max_distance) {
        max_distance = *it; // Update the maximum distance
        max_index = it - relevant_scan_ranges.begin(); // Update the index
      }
    }
    return max_index;
  }

  void get_safest_direction() {
    int max_distance_index = get_max_distance_index();
    int front_ray_index =
        static_cast<int>(relevant_scan_ranges.size()) / 2; // 180
    direction_ = (max_distance_index - front_ray_index) * angle_increment;
    // std::cout << "safest direction : " << direction_ << std::endl;
  }

  // we will verify existance of obstacles ahead using all the rays between
  // [-45°,45°] corresponding to relevant_scan_ranges[90:270]
  bool obstacle_detected_forward() {
    obstacle_detected = false;
    if (!relevant_scan_ranges.empty()) {
      for (auto it = relevant_scan_ranges.begin() + 90;
           it != relevant_scan_ranges.begin() + 270; it++) {
        if (*it < 0.35) {
          obstacle_detected = true;
          RCLCPP_INFO(this->get_logger(), "Obstacle detected ahead!!");
          break;
        }
      }
    }
    return obstacle_detected;
  }

  // move robot forward till detecting an obstacle less than 35cm
  // However, it is problematic to use only 1 ray to verify existence of
  // obstacles, instead we use obstacle_detected_forward method
  void timer_callback() {
<<<<<<< HEAD
    this->twist_msg.linear.x = 0.1;
    if (obstacle_detected_forward()) {
=======
    if (obstacle_detected) {
>>>>>>> 992f095 (Adapted the code to work on the real robot)
      this->twist_msg.angular.z = direction_ / 2.0;
      // RCLCPP_INFO(this->get_logger(), "TURNING....");

    } else {
      this->twist_msg.angular.z = 0.0;
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
  bool obstacle_detected;
  std::vector<float> relevant_scan_ranges;

  geometry_msgs::msg::Twist twist_msg;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_robot_publisher;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  // rclcpp::spin(std::make_shared<Patrol>());
  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

  // This is the same as a print in ROS
  RCLCPP_INFO(patrol_node->get_logger(), "Patrol Node STARTING ...");

  // Use mutithreading for more efficiency
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}