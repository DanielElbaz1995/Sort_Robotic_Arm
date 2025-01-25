#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "sort_robotic_arm_interface/msg/coordinates_list.hpp"

class ArmController : public rclcpp::Node {
public:
    ArmController();

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp::Subscription<sort_robotic_arm_interface::msg::CoordinatesList>::SharedPtr objects_subscriber_;
    rclcpp::TimerBase::SharedPtr sequence_timer_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> position_sequence_;
    size_t current_sequence_index_ = 0;
    bool movement_in_progress_ = false;
    bool sequence_in_progress_ = false;
    rclcpp::Time last_movement_time_;
    const double MOVEMENT_COMPLETION_TIME = 3.0;  // seconds to wait for movement completion
    float y_offset = 277.95;
    int width = 640;
    int changeThreshold = 10;

    void moveArm(const std::vector<double>& positions, double duration_sec = 2.0);
    void moveArmSequence(const std::vector<std::vector<double>>& position_sequence);
    void detectedObjectsCallback(const sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr msg);
    float RotateAngle(float x, float y);
    bool CheckForChanges(sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr objects,
                        sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr Newobjects);
    bool CheckForSpecificChange(const sort_robotic_arm_interface::msg::Coordinates &curr_obj,
                              const sort_robotic_arm_interface::msg::Coordinates &original_obj);
    void startNextMovement();
    void checkMovementCompletion();                               
};
#endif