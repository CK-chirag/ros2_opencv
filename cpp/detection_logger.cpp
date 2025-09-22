// detection_logger.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class DetectionLogger : public rclcpp::Node {
public:
  DetectionLogger(): Node("detection_logger") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/yolo/detections", 10,
      std::bind(&DetectionLogger::callback, this, _1));
  }
private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Detections: %s", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionLogger>());
  rclcpp::shutdown();
  return 0;
}
