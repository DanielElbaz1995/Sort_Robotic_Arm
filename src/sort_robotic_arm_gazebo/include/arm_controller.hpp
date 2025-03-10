#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <sort_robotic_arm_interface/action/send_angle.hpp>
#include <sort_robotic_arm_interface/action/send_goal.hpp>

using SendAngle = sort_robotic_arm_interface::action::SendAngle;
using SendAngleGoalHandle = rclcpp_action::ServerGoalHandle<SendAngle>;
using namespace std::placeholders;

class ArmController : public rclcpp::Node {
public:
    ArmController();

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp_action::Server<SendAngle>::SharedPtr angles_subscriber_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> position_sequence_;
    const double MOVEMENT_COMPLETION_TIME = 3.0;  // seconds to wait for movement completion
    double grip_open_angle = 0.785;
    double grip_close_angle = 0.28;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SendAngle::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<SendAngleGoalHandle> goal_handle);
    void handle_accepted_callback(const std::shared_ptr<SendAngleGoalHandle> goal_handle);
    void execute_goal(const std::shared_ptr<SendAngleGoalHandle> goal_handle);
    void moveArm(const std::vector<double>& positions, double duration_sec = 2.0);
    void MovementSequence(double alpha, double theta1 , double theta2, double theta3);                     
};
#endif
