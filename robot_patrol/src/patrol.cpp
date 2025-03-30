#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

using std::placeholders::_1;

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
  }

private:
  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // RCLCPP_INFO(this->get_logger(), " length of ranges: '%lx'",
    //             msg->ranges.size());
    std::cout << "MAX number of rays" << msg->ranges.max_size() << std::endl;
    std::cout << "number of rays" << msg->ranges.size() << std::endl;

    RCLCPP_INFO(this->get_logger(), " ranges[0]: '%f'", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), " ranges[165]: '%f'", msg->ranges[165]);
    RCLCPP_INFO(this->get_logger(), " ranges[330]: '%f'", msg->ranges[330]);
    RCLCPP_INFO(this->get_logger(), " ranges[495]: '%f'", msg->ranges[495]);
    RCLCPP_INFO(this->get_logger(), " ranges[660]: '%f'", msg->ranges[659]);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());

  rclcpp::shutdown();

  return 0;
}