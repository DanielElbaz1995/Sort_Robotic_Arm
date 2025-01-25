#include "object_detector.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // Initializes the ROS 2 system.
    auto node = std::make_shared<ObjectDetectorNode>(); 
    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}