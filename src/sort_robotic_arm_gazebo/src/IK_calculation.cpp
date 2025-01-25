#include <rclcpp/rclcpp.hpp>
#include <sort_robotic_arm_interface/msg/coordinates.hpp>
#include <sort_robotic_arm_interface/msg/coordinates_list.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <thread>

class IKCalculationNode : public rclcpp::Node {
public:
   IKCalculationNode() : Node("IK_calculation_node") {
        detected_objects_sub_ = this->create_subscription<sort_robotic_arm_interface::msg::Coordinates>(
           "detected_objects", 10, [this](sort_robotic_arm_interface::msg::Coordinates::SharedPtr msg) {
               x_target = msg->x;
               y_target = msg->y;
               calculate_ik();
           });
        trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
   }

private:
    rclcpp::Subscription<sort_robotic_arm_interface::msg::Coordinates>::SharedPtr detected_objects_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    int64_t x_target;
    int64_t y_target;
    double y_offset = 162;
    double x_width_half = 320;
    //double m_to_pixels = 135; // we checked with cube size 1.2 and came width of 162 : (162/1.2 = 135)
    double arm1 = 162; //1.2 * m_to_pixels;
    double arm2 = 160.65; //1.199 * m_to_pixels;
    double arm3 = 164.16; //1.216 * m_to_pixels;
    double base_height = 131.86;  //0.9768 * m_to_pixels;

    std::vector<std::string> joint_names_ = {
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

    void moveArm(const std::vector<double>& positions, double duration_sec) {
            auto rotation = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
            rotation->joint_names = joint_names_;

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;
            point.time_from_start.sec = static_cast<int>(duration_sec);
            point.time_from_start.nanosec = static_cast<uint32_t>((duration_sec - static_cast<int>(duration_sec)) * 1e9);

            rotation->points.push_back(point);
            trajectory_publisher_->publish(*rotation);
    }

    bool ObjectTooFarFor90Deg(double radIn3D){
        if ((arm2 + arm1)<radIn3D)
            return true;
        else return false;
    }   

    void CalcAnglesforArmin90Deg(double *beta1, double *beta2, double *beta3, double radIn2D, double radIn3D){
        double theta1; //triangle angle 1
        double theta2; //triangle angle 2
        double theta3; //triangle angle 3
        double diffAngleBaseArm3in90deg = std::atan2(arm3 - base_height, radIn2D);
        theta3 = std::acos((std::pow(radIn3D, 2) + std::pow(arm2, 2) - std::pow(arm1, 2)) / (2 * radIn3D * arm2));
        theta2 = std::acos((std::pow(arm2, 2) + std::pow(arm1, 2) - std::pow(radIn3D, 2)) / (2 * arm1 * arm2));
        theta1 = ((M_PI - theta2) - theta3);
        *beta2 = theta2 - M_PI;
        *beta3 = theta3 - M_PI/2;
        *beta1 = (theta1 + diffAngleBaseArm3in90deg) - (M_PI / 4);

    }

    void calculate_ik() {
        double alpha; //alpha is the angle of servo 1 (base)
        double beta1; //beta1 is the angle of servo 2 (arm1)
        double beta2; //beta2 is the angle of servo 2 (arm2)
        double beta3; //beta3 is the angle of servo 2 (arm3)
        double radIn2D = std::sqrt(std::pow(x_target - x_width_half, 2) + std::pow(y_target + y_offset, 2));
        double radIn3D = std::sqrt(std::pow(base_height - arm3, 2) + std::pow(radIn2D, 2));
        double grip_open_angle = 0.785;
        double grip_close_angle = 0.39;
        alpha = std::atan2(x_target-x_width_half, y_target + y_offset);
        if(ObjectTooFarFor90Deg(radIn3D)){
            beta1 = 0;
            beta2 = 0;
            beta3 = 0;
        }
        else{ //arm3 goes to 90 deg
            CalcAnglesforArmin90Deg(&beta1, &beta2, &beta3, radIn2D, radIn3D);
        }
        RCLCPP_INFO(this->get_logger(), "beta1 = %.2f", beta1);
        RCLCPP_INFO(this->get_logger(), "beta2 = %.2f", beta2);
        RCLCPP_INFO(this->get_logger(), "beta3 = %.2f", beta3);
        
        std::vector<double> position_sequence = {
         alpha, beta1, beta2, 1.545, beta3, grip_open_angle, grip_open_angle,
         grip_open_angle, grip_open_angle, grip_open_angle, grip_open_angle,
         grip_open_angle, grip_open_angle};
        moveArm(position_sequence, 10.0);
        std::this_thread::sleep_for(std::chrono::seconds(15));
        position_sequence = {
         alpha, beta1, beta2, 1.545, beta3, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle, grip_close_angle};
        moveArm(position_sequence, 10.0);
    }
};

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<IKCalculationNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}