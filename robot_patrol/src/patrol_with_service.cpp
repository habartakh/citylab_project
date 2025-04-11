#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "robot_patrol/srv/get_direction.hpp"

#define M_PI_6 0.5235983

using std::placeholders::_1;
using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_node") {

    // Create a subscriber to /scan topic
    scan_sub_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = scan_sub_callback_group_;
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

    client_ = this->create_client<GetDirection>("direction_service");
  }

private:
  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_scan_msg = *msg;
    bool obstacle_detected = false;

    int current_index = 0;
    for (auto it = msg->ranges.begin(); it != msg->ranges.end(); ++it) {

      float current_angle =
          msg->angle_min + current_index * msg->angle_increment;
      // obstacle detection in front of the robot between[-pi/6,pi/6]
      if (current_angle >= -M_PI_4 && current_angle <= M_PI_4) {
        if (std::isfinite(*it) && *it <= 0.35) {
          obstacle_detected = true;
        }
      }
      current_index++;
    }

    this->twist_msg.angular.z = 0.0;
    this->twist_msg.linear.x = 0.1;

    if (obstacle_detected) {
      send_async_request(); // call the service to get next direction to move to

      if (direction_ == "forward") {
        this->twist_msg.angular.z = 0.0;
      }

      if (direction_ == "right") {
        this->twist_msg.angular.z = -0.5;
      }
      if (direction_ == "left") {
        this->twist_msg.angular.z = 0.5;
      }
    }

    move_robot_publisher->publish(this->twist_msg);
  }

  // move robot forward till detecting an obstacle less than 35cm
  // However, it is problematic to use only 1 ray to verify existence of
  // obstacles, instead we use obstacle_detected_forward method
  void timer_callback() {}

  void send_async_request() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = laser_scan_msg;
    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::response_callback, this, std::placeholders::_1));

    // Now check for the response after a timeout of 1 second
    auto status = result_future.wait_for(1s);

    if (status != std::future_status::ready) {

      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }

  // will be called when the service returns a response
  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    // Get response value
    auto response = future.get();
    direction_ = response->direction;
    RCLCPP_INFO(this->get_logger(), "Response: %s \n",
                response->direction.c_str());
    service_done_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  rclcpp::CallbackGroup::SharedPtr scan_sub_callback_group_;
  rclcpp::SubscriptionOptions options;

  rclcpp::Client<GetDirection>::SharedPtr client_;
  sensor_msgs::msg::LaserScan laser_scan_msg;

  std::string direction_; // safest direction robot can take to complete patrol
  bool service_done_ = false;

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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}