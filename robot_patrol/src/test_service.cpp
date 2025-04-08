#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using GetDirection = robot_patrol::srv::GetDirection;

class TestServiceNode : public rclcpp::Node {
private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  sensor_msgs::msg::LaserScan laser_scan_msg;

  bool service_done_ = false;
  bool service_called_ = false;

  void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    laser_scan_msg = *msg;
  }

  void timer_callback() {

    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Service Requested");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Please wait a moment...");
    }
  }

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

    // RCLCPP_INFO(this->get_logger(), "Service Request \n");
    auto result_future = client_->async_send_request(
        request, std::bind(&TestServiceNode::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;

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
    RCLCPP_INFO(this->get_logger(), "Service returned response: %s \n",
                response->direction.c_str());
    service_done_ = true;
  }

public:
  TestServiceNode() : Node("service_client") {
    client_ = this->create_client<GetDirection>("direction_service");
    RCLCPP_INFO(this->get_logger(), "Service Client Ready \n");

    timer_ = this->create_wall_timer(
        1s, std::bind(&TestServiceNode::timer_callback, this));

    laser_scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&TestServiceNode::scan_topic_callback, this, _1));
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<TestServiceNode>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}
