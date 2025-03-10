#ifndef IK_CALCULATION_HPP
#define IK_CALCULATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <thread>

#include <rclcpp_action/rclcpp_action.hpp>
#include <sort_robotic_arm_interface/action/send_goal.hpp>
#include <sort_robotic_arm_interface/action/send_angle.hpp>

using SendGoal = sort_robotic_arm_interface::action::SendGoal;
using SendGoalHandle = rclcpp_action::ServerGoalHandle<SendGoal>;
using SendAngle = sort_robotic_arm_interface::action::SendAngle;
using SendAngleGoalHandle = rclcpp_action::ClientGoalHandle<SendAngle>;
using namespace std::placeholders;

class IKCalculationNode : public rclcpp::Node {
public:
    IKCalculationNode();
    
private:
    rclcpp_action::Server<SendGoal>::SharedPtr detected_objects_;
    rclcpp_action::Client<SendAngle>::SharedPtr angles_publisher_;
    //rclcpp::Client<gazebo_msgs::srv::CreateJoint>::SharedPtr create_joint_client_;
    int64_t x_target;
    int64_t y_target;
    double y_offset = 162;
    double x_width_half = 640 / 2;
    int m_per_pixel = 135; // Pixels factor

    // Link lengths
    double arm1 = 1.2 * m_per_pixel;
    double arm2 = 1.199 * m_per_pixel;
    double arm3 = 1.216 * m_per_pixel;
    double base_height = 0.9 * m_per_pixel; // (base_offset - gripper_end_offset) 0.9368

    bool ObjectTooFarFor90Deg(double radIn3D);
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SendGoal::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<SendGoalHandle> goal_handle);
    void handle_accepted_callback(const std::shared_ptr<SendGoalHandle> goal_handle);
    void execute_goal(const std::shared_ptr<SendGoalHandle> goal_handle);
    void calculate_ik(int x_target, int y_target, const std::shared_ptr<SendGoalHandle> goal_handle);
    void SendAngles(double alpha, double theta1, double theta2, double theta3, const std::shared_ptr<SendGoalHandle> goal_handle);
    void goal_result_callback(const SendAngleGoalHandle::WrappedResult &result, const std::shared_ptr<SendGoalHandle> goal_handle);
};
#endif
