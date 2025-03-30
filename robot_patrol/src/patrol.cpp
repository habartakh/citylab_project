#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

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
    // print number of laser rays = 294
    RCLCPP_INFO(this->get_logger(), " length of ranges: '%lx'",
                msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), " length of intensities: '%lx'",
                msg->intensities.size());

    // RCLCPP_INFO(this->get_logger(), " angle_min : '%f'", msg->angle_min);
    // RCLCPP_INFO(this->get_logger(), " angle_max : '%f'", msg->angle_max);

    RCLCPP_INFO(this->get_logger(), " ranges[0]: '%f'", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), " ranges[73]: '%f'", msg->ranges[73]);
    RCLCPP_INFO(this->get_logger(), " ranges[146]: '%f'", msg->ranges[146]);
    RCLCPP_INFO(this->get_logger(), " ranges[219]: '%f'", msg->ranges[219]);
    RCLCPP_INFO(this->get_logger(), " ranges[293]: '%f'", msg->ranges[293]);
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