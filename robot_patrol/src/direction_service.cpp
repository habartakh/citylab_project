#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
// #include <limits>

#include <memory>

#include "robot_patrol/srv/get_direction.hpp"

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {

public:
  DirectionService() : Node("get_direction_service") {
    srv_ = create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::service_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service Server Ready \n");
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  float total_dist_sec_right;
  float total_dist_sec_front;
  float total_dist_sec_left;

  void
  service_callback(const std::shared_ptr<GetDirection::Request> request,
                   const std::shared_ptr<GetDirection::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Service Server Requested \n");

    // Index in the Laser_data corresponding to -M_PI_2
    int index_minus_pi_2 =
        static_cast<int>(std::abs((-M_PI_2 - request->laser_data.angle_min) /
                                  request->laser_data.angle_increment));

    int index_plus_pi_2 =
        static_cast<int>(std::abs((M_PI_2 - request->laser_data.angle_min) /
                                  request->laser_data.angle_increment));

    int index_increment =
        static_cast<int>((index_plus_pi_2 - index_minus_pi_2) / 3);

    // std::cout << "index_minus_pi_2 : " << index_minus_pi_2 << std::endl;
    // std::cout << "index_plus_pi_2 : " << index_plus_pi_2 << std::endl;
    // std::cout << "index_increment : " << index_increment << std::endl;

    total_dist_sec_right = get_section_distance(
        request->laser_data.ranges.begin() + index_minus_pi_2,
        request->laser_data.ranges.begin() + index_minus_pi_2 +
            index_increment);
    total_dist_sec_front = get_section_distance(
        request->laser_data.ranges.begin() + index_minus_pi_2 + index_increment,
        request->laser_data.ranges.begin() + index_plus_pi_2 - index_increment);
    total_dist_sec_left = get_section_distance(
        request->laser_data.ranges.begin() + index_plus_pi_2 - index_increment,
        request->laser_data.ranges.begin() + index_plus_pi_2);

    // std::cout << "total_dist_sec_right: " << total_dist_sec_right <<
    // std::endl; std::cout << "total_dist_sec_front: " << total_dist_sec_front
    // << std::endl; std::cout << "total_dist_sec_left:  " <<
    // total_dist_sec_left << std::endl;

    if (total_dist_sec_right >= total_dist_sec_front &&
        total_dist_sec_right >= total_dist_sec_left) {
      response->direction = "right";
    }
    if (total_dist_sec_front >= total_dist_sec_right &&
        total_dist_sec_front >= total_dist_sec_left) {
      response->direction = "front";
    }
    if (total_dist_sec_left >= total_dist_sec_front &&
        total_dist_sec_left >= total_dist_sec_right) {
      response->direction = "left";
    }
    RCLCPP_INFO(this->get_logger(), "Service returned: %s \n",
                response->direction.c_str());
    RCLCPP_INFO(this->get_logger(), "Service Completed \n");
  }

  // Get the total distances for different sections of 60° of the laser rays
  float get_section_distance(auto begin_iterator, auto end_iterator) {
    float section_sum = 0.0;
    for (auto it = begin_iterator; it < end_iterator + 1; it++) {
      if (std::isfinite(*it)) {
        section_sum += *it;
      }
    }
    return section_sum;
  }
};
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}