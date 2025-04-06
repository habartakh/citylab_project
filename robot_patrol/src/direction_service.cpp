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
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  float total_dist_sec_right;
  float total_dist_sec_front;
  float total_dist_sec_left;

  void
  service_callback(const std::shared_ptr<GetDirection::Request> request,
                   const std::shared_ptr<GetDirection::Response> response) {

    total_dist_sec_right =
        get_section_distance(request->laser_data.ranges.begin() + 165,
                             request->laser_data.ranges.begin() + 275);
    total_dist_sec_front =
        get_section_distance(request->laser_data.ranges.begin() + 275,
                             request->laser_data.ranges.begin() + 385);
    total_dist_sec_left =
        get_section_distance(request->laser_data.ranges.begin() + 385,
                             request->laser_data.ranges.begin() + 495);

    std::cout << "total_dist_sec_right" << total_dist_sec_right << std::endl;
    std::cout << "total_dist_sec_front" << total_dist_sec_front << std::endl;
    std::cout << "total_dist_sec_left" << total_dist_sec_left << std::endl;

    if (total_dist_sec_right >= total_dist_sec_front && total_dist_sec_right >= total_dist_sec_left){
        response->direction = "right";
    }
    if (total_dist_sec_front >= total_dist_sec_right && total_dist_sec_front >= total_dist_sec_left){
        response->direction = "front";
    }
    if (total_dist_sec_left >= total_dist_sec_front && total_dist_sec_left >= total_dist_sec_right){
        response->direction = "left";
    }
  }

  // Get the average distance for different sections of 60° of the laser rays
  float get_section_distance(auto begin_iterator, auto end_iterator) {
    float section_sum = 0.0;
    int number_of_elements = 0;
    for (auto it = begin_iterator; it < end_iterator + 1; it++) {
      if (std::isfinite(*it)) {
        section_sum += *it;
        number_of_elements++;
      }
    }
    return section_sum / number_of_elements;
  }
};
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}