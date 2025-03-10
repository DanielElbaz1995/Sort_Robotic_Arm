#include "arm_controller.hpp"

ArmController::ArmController() : Node("arm_controller_node") {
    angles_subscriber_ = rclcpp_action::create_server<SendAngle>(
            this,
            "angles_moves",
            std::bind(&ArmController::goal_callback, this, _1, _2),
            std::bind(&ArmController::cancel_callback, this, _1),
            std::bind(&ArmController::handle_accepted_callback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "ArmController Server has been started");

    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    joint_names_ = {
        "servo1_waist_joint",
        "servo2_arm1_joint",
        "arm1_servo3_joint",
        "micro_servo1_arm3_joint",
        "micro_servo2_gripper_base_joint",
        "micro_servo3_gear2_joint",
        "gripper_base_gear1_joint",
        "gripper_base_up_right_gripper_link_joint",
        "gripper_base_up_left_gripper_link_joint",
        "gripper_base_down_right_gripper_link_joint",
        "gripper_base_down_left_gripper_link_joint",
        "gear2_right_gripper_joint",
        "gear1_left_gripper_joint",
    };

}

rclcpp_action::GoalResponse ArmController::goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SendAngle::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Recieved the angles");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

rclcpp_action::CancelResponse ArmController::cancel_callback(const std::shared_ptr<SendAngleGoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmController::handle_accepted_callback(const std::shared_ptr<SendAngleGoalHandle> goal_handle) {
    execute_goal(goal_handle);
}

void ArmController::execute_goal(const std::shared_ptr<SendAngleGoalHandle> goal_handle) {
    // Get request from goal
    double alpha = goal_handle->get_goal()->alpha;
    double theta1 = goal_handle->get_goal()->theta1;
    double theta2 = goal_handle->get_goal()->theta2;
    double theta3 = goal_handle->get_goal()->theta3;
    
    // Execute the action
    MovementSequence(alpha, theta1 , theta2, theta3);

    // Set final state and return results
    auto result =std::make_shared<SendAngle::Result>();
    result->completed_angles = "Movement Done!";
    goal_handle->succeed(result);
}
void ArmController::moveArm(const std::vector<double>& positions, double duration_sec) {
    auto rotation = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    rotation->joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start.sec = static_cast<int>(duration_sec);
    point.time_from_start.nanosec = static_cast<uint32_t>((duration_sec - static_cast<int>(duration_sec)) * 1e9);

    rotation->points.push_back(point);
    trajectory_publisher_->publish(*rotation);
}

void ArmController::MovementSequence(double alpha, double theta1 , double theta2, double theta3) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::vector<double> position_sequence = {
        alpha, theta1, theta2, 1.545, theta3, grip_open_angle, grip_open_angle,
        grip_open_angle, grip_open_angle, grip_open_angle, grip_open_angle,
        grip_open_angle, grip_open_angle};
    moveArm(position_sequence, 1.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    position_sequence = {
        alpha, theta1, theta2, 1.545, theta3, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle};
    moveArm(position_sequence, 1.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    position_sequence = {
        alpha, theta1+0.5, theta2, 1.545, theta3, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle};
    moveArm(position_sequence, 1.0);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();
    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}
