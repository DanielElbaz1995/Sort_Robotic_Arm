#include "arm_controller.hpp"

ArmController::ArmController() : Node("arm_controller_node") {
    objects_subscriber_ = this->create_subscription<sort_robotic_arm_interface::msg::CoordinatesList>(
        "detected_objects", 10,
        std::bind(&ArmController::detectedObjectsCallback, this, std::placeholders::_1));

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

    sequence_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Check more frequently
        std::bind(&ArmController::checkMovementCompletion, this));
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

    movement_in_progress_ = true;
    last_movement_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Executing movement %zu of %zu", 
                current_sequence_index_ + 1, position_sequence_.size());
}

void ArmController::moveArmSequence(const std::vector<std::vector<double>>& position_sequence) {
    if (sequence_in_progress_) {
        RCLCPP_WARN(this->get_logger(), "Sequence already in progress, ignoring new request");
        return;
    }
    if (position_sequence.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty position sequence provided");
        return;
    }
    sequence_in_progress_ = true;
    movement_in_progress_ = false;
    position_sequence_ = position_sequence;
    current_sequence_index_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "Starting new sequence with %zu positions", position_sequence_.size());
    startNextMovement();
}

void ArmController::startNextMovement() {
    if (current_sequence_index_ >= position_sequence_.size()) {
        RCLCPP_INFO(this->get_logger(), "Sequence completed");
        sequence_in_progress_ = false;
        return;
    }
    moveArm(position_sequence_[current_sequence_index_]);
}

void ArmController::checkMovementCompletion() {
    if (!sequence_in_progress_ || !movement_in_progress_) {
        return;
    }
    rclcpp::Time current_time = this->now();
    double elapsed = (current_time - last_movement_time_).seconds();
    
    if (elapsed >= MOVEMENT_COMPLETION_TIME) {
        movement_in_progress_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Movement completed, elapsed time: %.2f seconds", elapsed);

        if (current_sequence_index_ >= position_sequence_.size()) {
            RCLCPP_INFO(this->get_logger(), "Full sequence completed");
            sequence_in_progress_ = false;
            return;
        }
        
        // Start next movement
        startNextMovement();
        current_sequence_index_++;
    }
}

void ArmController::detectedObjectsCallback(const sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr objects) {
    if (sequence_in_progress_) {
        return;
    }

    double grip=0.36;
    std::vector<std::vector<double>> position_sequence = {
        // Position 1: Initial position
        {0.666787, 0, 0, 1.545, 0, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785},
        // // Position 2: Close the gripper
        // {0.14, -0.136, -1.890, 1.545, 0.916, grip, grip, grip, grip, grip, grip, grip, grip},
        // // Position 3: Grip position
        // {0.14, 0.200, -1.890, 1.545, 0.916, grip, grip, grip, grip, grip, grip, grip, grip},
        // // Position 4: Return to rest
        // {0.14, 0.200, -1.890, 1.545, 0.916, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35}
        // // Position 1: Initial position
        // {0.14, -0.136, -1.890, 1.545, 0.916, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785},//, 0.785, 0.785},
        // // Position 2: Intermediate position
        // {0.14, -0.136, -1.890, 1.545, 0.916, 0.37, 0.37, 0.37, 0.37, 0.37, 0.37},//, 0.37, 0.37},
        // // Position 3: Grip position
        // {0.14, 0.200, -1.890, 1.545, 0.916, 0.37, 0.37, 0.37, 0.37, 0.37, 0.37},//, 0.37, 0.37},
        // // Position 4: Return to rest
        // {0.14, 0.200, -1.890, 1.545, 0.916, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35},//, 0.35, 0.35}
    };

    moveArmSequence(position_sequence);
}

// Your existing utility functions remain unchanged
float ArmController::RotateAngle(float x, float y) {
    float x_tilda = x - width/2;
    float y_tilda = y + y_offset;
    return atan2(x_tilda, y_tilda);
}

bool ArmController::CheckForChanges(sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr objects,
                                  sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr new_objects) {
    if (objects->points.size() != new_objects->points.size()) {
        return true;
    }
    for (size_t i = 0; i < objects->points.size(); i++) {
        if (CheckForSpecificChange(objects->points[i], new_objects->points[i])) {
            return true;
        }
    }
    return false;
}

bool ArmController::CheckForSpecificChange(const sort_robotic_arm_interface::msg::Coordinates &curr_obj,
                                         const sort_robotic_arm_interface::msg::Coordinates &original_obj) {
    int64_t x_diff = curr_obj.x - original_obj.x;
    int64_t y_diff = curr_obj.y - original_obj.y;
    return (std::abs(x_diff) > changeThreshold || std::abs(y_diff) > changeThreshold);
}