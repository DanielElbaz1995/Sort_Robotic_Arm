#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "sort_robotic_arm_interface/msg/coordinates.hpp"
#include "sort_robotic_arm_interface/msg/coordinates_list.hpp"
#include <thread>

class ObjectDetectorNode : public rclcpp::Node {
public:
    ObjectDetectorNode();

private:
    rclcpp::Publisher<sort_robotic_arm_interface::msg::Coordinates>::SharedPtr objects_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::vector<std::pair<int, int>> AllCircles;
    std::vector<std::pair<int, int>> AllSquares;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool detectObjects(const sensor_msgs::msg::Image::SharedPtr raw_frame);
    void processCircle(const cv::Vec3f& circle, cv::Mat& frame);
    void processCube(const std::vector<cv::Point>& contour, cv::Mat& frame);
    std::pair<int, int> CheckForCube(const std::vector<cv::Point>& contour);
    void DetectCircles (cv::Mat& frame, cv::Mat& gray_frame);
    void DetectCubes (cv::Mat& frame, cv::Mat& gray_frame);
    void WaitOneSecond();
    void WaitForResponse();
    void SendObjects();
};

#endif