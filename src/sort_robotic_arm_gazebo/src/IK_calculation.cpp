#include "IK_calculation.hpp"

IKCalculationNode::IKCalculationNode() : Node("IK_calculation_node") {
    detected_objects_ = rclcpp_action::create_server<SendGoal>(
        this,
        "detected_objects",
        std::bind(&IKCalculationNode::goal_callback, this, _1, _2),
        std::bind(&IKCalculationNode::cancel_callback, this, _1),
        std::bind(&IKCalculationNode::handle_accepted_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "IK_calculation Server has been started");

    angles_publisher_ = rclcpp_action::create_client<SendAngle>(this, "angles_moves");
    RCLCPP_INFO(this->get_logger(), "IK_calculation Client has been started");
}

bool IKCalculationNode::ObjectTooFarFor90Deg(double radIn3D){
    if ((arm2 + arm1) < radIn3D)
        return true;
    else return false;
}   

rclcpp_action::GoalResponse IKCalculationNode::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SendGoal::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Recieved a goal");
    int x_target = goal->x;
    int y_target = goal->y;
    double radIn2D = std::sqrt(std::pow(x_target - x_width_half, 2) + std::pow(y_target + y_offset, 2));
    double radIn3D = std::sqrt(std::pow(base_height - arm3, 2) + std::pow(radIn2D, 2));
    if(ObjectTooFarFor90Deg(radIn3D)) {
        RCLCPP_INFO(this->get_logger(), "Rejecting the goal: Target too far");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepting the goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IKCalculationNode::cancel_callback(const std::shared_ptr<SendGoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void IKCalculationNode::handle_accepted_callback(const std::shared_ptr<SendGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void IKCalculationNode::execute_goal(const std::shared_ptr<SendGoalHandle> goal_handle) {
    // Get request from goal
    int x_target = goal_handle->get_goal()->x;
    int y_target = goal_handle->get_goal()->y;
    
    // Execute the action
    calculate_ik(x_target, y_target, goal_handle);
}

void IKCalculationNode::calculate_ik(int x_target, int y_target, const std::shared_ptr<SendGoalHandle> goal_handle) {
    double alpha = std::atan2(x_target - x_width_half, y_target + y_offset);

    // New target x-z plane
    x_target = std::sqrt(std::pow(x_target - x_width_half, 2) + std::pow(y_target + y_offset, 2));
    double z_target = -base_height;
    double psi = -M_PI / 2;

    // Calculate earlier point
    double x = x_target - arm3 * std::cos(psi);
    double z = z_target - arm3 * std::sin(psi);

    // Using law of cosines for second joint
    double cos_theta2 = (pow(x, 2) + pow(z, 2) - pow(arm1, 2) - pow(arm2, 2)) / (2 * arm1 * arm2);
    double theta2 = -std::acos(cos_theta2);
    
    // Calculate theta1 using geometry
    double beta = std::atan2((arm2 * std::sin(theta2)) , (arm1 + (arm2 * std::cos(theta2))));
    double gamma = std::atan2(z, x);
    double theta1 = ((gamma - beta) - M_PI / 4);
    
    // Calculate theta3 to keep end-effector horizontal
    double theta3 = psi - (theta1 + theta2) - M_PI / 4;

    SendAngles(alpha, theta1, theta2, theta3, goal_handle);
}

void IKCalculationNode::SendAngles(double alpha, double theta1, double theta2, double theta3,
                                    const std::shared_ptr<SendGoalHandle> goal_handle) 
{
    // Wait for the action server
    angles_publisher_->wait_for_action_server();

    // Create a goal
    auto goal = SendAngle::Goal();
    goal.alpha = alpha;
    goal.theta1 = theta1;
    goal.theta2 = theta2;
    goal.theta3 = theta3;

    // Add callback
    auto options = rclcpp_action::Client<SendAngle>::SendGoalOptions();
    options.result_callback = [this, goal_handle](const SendAngleGoalHandle::WrappedResult &result) {
        goal_result_callback(result, goal_handle);
    };

    // Send the goal
    angles_publisher_->async_send_goal(goal, options);
}

void IKCalculationNode::goal_result_callback(const SendAngleGoalHandle::WrappedResult &result,
                                             const std::shared_ptr<SendGoalHandle> goal_handle)
{
    std::string goal_achieved = result.result->completed_angles;
    RCLCPP_INFO(this->get_logger(), "%s", goal_achieved.c_str());

    // Set final state and return results
    auto final_result  = std::make_shared<SendGoal::Result>();
    final_result->goal_achieved = "Goal Achieved!";
    goal_handle->succeed(final_result);
}

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<IKCalculationNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
