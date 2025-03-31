#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <limits>
#include <vector>

#define PI 3.14159
using std::placeholders::_1;
using namespace std::chrono_literals;

// Number of laser rays : 660
// Laser scan ranges vector correspondances:
// 0-360 : BACK
// 165   : RIGHT
// 330   : FRONT
// 465   : LEFT

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_node") {

    // Create a subscriber to /scan topic
    laser_scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Patrol::scan_topic_callback, this, _1));

    // create a publisher to move the robot
    move_robot_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&Patrol::timer_callback, this));
  }

private:
 
  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    distance_front = msg->ranges[330];      // distance in front of the robot
    range_max = msg->range_max;             // maximum range value [m]
    angle_increment = msg->angle_increment; // angular distance b/w laser scan
                                            // measurements [rad]

    // we will only work with the 180 degrees laser rays in front of the robot
    // thus, relevant_scan_ranges[165:495] corresponding to [-pi/2,pi/2] rays
    relevant_scan_ranges = std::vector<float>(msg->ranges.begin() + 165,
                                              msg->ranges.begin() + 495);

    get_safest_direction();
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

    int index_max_distance = get_max_distance_index();
    direction_ = -(165 - index_max_distance) * angle_increment;
    std::cout << "safest direction : " << direction_ << std::endl;
  }

  void timer_callback() {

    // move robot forward till detecting an obstacle less than 35cm
    if (distance_front >= 0.35) {
      this->twist_msg.angular.z = 0.0;

    } else {

      this->twist_msg.angular.z += direction_ / 2.0;
      //   std::cout << "this->twist_msg.angular.z" << this->twist_msg.angular.z
      //             << std::endl;
    }
    this->twist_msg.linear.x = 0.1;
    move_robot_publisher->publish(this->twist_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  float direction_; // safest direction robot can take to complete patrol
  float distance_front;
  float range_max;
  float angle_increment;
  std::vector<float> relevant_scan_ranges;

  geometry_msgs::msg::Twist twist_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_robot_publisher;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());

  rclcpp::shutdown();

  return 0;
}