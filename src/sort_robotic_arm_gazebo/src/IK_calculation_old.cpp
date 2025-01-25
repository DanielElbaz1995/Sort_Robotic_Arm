#include <rclcpp/rclcpp.hpp>
#include <sort_robotic_arm_interface/msg/coordinates.hpp>
#include <sort_robotic_arm_interface/msg/coordinates_list.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class IKCalculationNode : public rclcpp::Node {
public:
   IKCalculationNode() : Node("IK_calculation_node") {
       detected_objects_sub_ = this->create_subscription<sort_robotic_arm_interface::msg::CoordinatesList>(
           "detected_objects", 10, [this](sort_robotic_arm_interface::msg::CoordinatesList::SharedPtr msg) {
               x_target = msg->points[0].x;
               y_target = msg->points[0].y;
               calculate_ik_elbow_up();
           });
   }

private:

   rclcpp::Subscription<sort_robotic_arm_interface::msg::CoordinatesList>::SharedPtr detected_objects_sub_;
   int64_t x_target;
   int64_t y_target;

   void calculate_ik_elbow_up() {
       double m_to_pixels = 16 / 0.1;
       double arm1 = 1.2 * m_to_pixels;
       double arm2 = 1.199 * m_to_pixels;
       double arm3 = 1.216 * m_to_pixels;
       double x_width = 640 / 2;
       double y_offset = 277.95;

       double psi = 0; //-M_PI / 4; 
       double x = std::sqrt(std::pow(x_target - x_width, 2) + std::pow(y_target + y_offset, 2));
       x = x - arm3 * std::cos(psi);
       double z = 0;
       z = z + arm3 * std::sin(psi);
       

       double r = std::sqrt(std::pow(x, 2) + std::pow(z, 2));
       double cos_theta2 = (std::pow(x, 2) + std::pow(z, 2) - std::pow(arm1, 2) - std::pow(arm2, 2)) / (2 * arm1 * arm2);
       double theta2 = std::acos(cos_theta2);

       double beta = std::atan2(z, x);
       double gamma = std::acos((std::pow(arm1, 2) + std::pow(r, 2) - std::pow(arm2, 2)) / (2 * arm1 * r));
       double theta1 = beta - gamma;

       double theta3 = psi - (theta1 + theta2);

       RCLCPP_INFO(this->get_logger(), "Joint angles (rad):");
       RCLCPP_INFO(this->get_logger(), "θ1 = %.2f", theta1);
       RCLCPP_INFO(this->get_logger(), "θ2 = %.2f", theta2);
       RCLCPP_INFO(this->get_logger(), "θ3 = %.2f", theta3);
   }
};

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<IKCalculationNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}