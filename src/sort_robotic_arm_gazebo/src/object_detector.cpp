#include "object_detector.hpp"

ObjectDetectorNode::ObjectDetectorNode() : Node("object_detector_node") {  
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ObjectDetectorNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Creating image publisher");

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>
        ("processed_image", 10);
    RCLCPP_INFO(this->get_logger(), "Creating coordinates publisher");

    objects_pub_ = rclcpp_action::create_client<SendGoal>(this, "detected_objects");
    RCLCPP_INFO(this->get_logger(), "Created action client");
}

void ObjectDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr raw_frame) {
    bool foundObjects = detectObjects(raw_frame);
    if (foundObjects){
        SendObjects();
        // WaitForResponse();
    }
    else{
        // WaitOneSecond();
    }
}

bool ObjectDetectorNode::detectObjects(const sensor_msgs::msg::Image::SharedPtr raw_frame) {
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_frame, "bgr8");
        cv::Mat frame = cv_ptr->image;
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        DetectCircles(frame, gray_frame);
        DetectCubes(frame, gray_frame);

        sensor_msgs::msg::Image::SharedPtr out_msg = 
            cv_bridge::CvImage(raw_frame->header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*out_msg);
    }
    catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    if ((AllCircles.size() == 0) && (AllSquares.size() == 0))
        return false;
    return true;
}

void ObjectDetectorNode::DetectCircles (cv::Mat& frame, cv::Mat& gray_frame){
    std::vector<cv::Vec3f> circles;
    AllCircles.clear();
    cv::HoughCircles(gray_frame, circles, cv::HOUGH_GRADIENT, 1,
                    gray_frame.rows / 8, 100, 20, 0.001, 100);
                    
    for(const auto& circle : circles) {
        std::pair<int, int> circleCoor = {cvRound(circle[0]), cvRound(circle[1])};
        AllCircles.push_back(circleCoor);
        processCircle(circle, frame);
    }
}

void ObjectDetectorNode::DetectCubes (cv::Mat& frame, cv::Mat& gray_frame){
    cv::Mat edges;
    std::vector<std::vector<cv::Point>> contours;
    AllSquares.clear();
    cv::Canny(gray_frame, edges, 0, 120);
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for(const auto& contour : contours) {
        std::pair<int, int> squareCoor = CheckForCube(contour);
        if (squareCoor.first != -1){ // -1 says its not a cube
            processCube(contour, frame);
            AllSquares.push_back(squareCoor);
        }
    }
}

void ObjectDetectorNode::processCircle(const cv::Vec3f& circle, cv::Mat& frame) {
    cv::Point center(circle[0], circle[1]);
    int radius = circle[2];
    cv::circle(frame, center, radius, cv::Scalar(0,0,255), 1);
}

void ObjectDetectorNode::processCube(const std::vector<cv::Point>& contour, cv::Mat& frame) {
    std::vector<cv::Point> approx;
    double epsilon = 0.04 * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);
    cv::drawContours(frame, std::vector<std::vector<cv::Point>>{approx}, 
                     0, cv::Scalar(0,255,0), 1);
}

std::pair<int, int> ObjectDetectorNode::CheckForCube(const std::vector<cv::Point>& contour){
    std::vector<cv::Point> approx;
    double epsilon = 0.04 * cv::arcLength(contour, true); // Calculate error possibilty
    cv::approxPolyDP(contour, approx, epsilon, true); // Approx polygon sides
    
    if(approx.size() == 4 && cv::isContourConvex(approx)) {
        double area = cv::contourArea(approx);
        if (area > 10 && area < 1000) { // Ignore noises, and not too much big squares
            cv::Rect rect = cv::boundingRect(approx);
            float aspect_ratio = (float)rect.width / rect.height; // Check if the sides equal
            if(aspect_ratio >= 0.8 && aspect_ratio <= 1.2) {
                int x_center = rect.x + rect.width / 2;
                int y_center = rect.y + rect.height / 2;
                //RCLCPP_INFO(this->get_logger(), "Rect Width: %d", rect.width);
                return {x_center, y_center};
            }
        }
    }
    return {-1,-1};
}

void ObjectDetectorNode::SendObjects() {
    static bool toDelete = false;
    // Wait for the action server
    objects_pub_->wait_for_action_server();

    // Create a goal
    auto goal = SendGoal::Goal();
    if (toDelete) return;
    if ((AllCircles.size() > 0)){
        goal.x = AllCircles[0].first;
        goal.y = AllCircles[0].second;
    }
    else{ // AllSquars.size() > 0
        goal.x = AllSquares[0].first;
        goal.y = AllSquares[0].second;
    }

    // Add callback
    auto options = rclcpp_action::Client<SendGoal>::SendGoalOptions();
    options.result_callback = std::bind(&ObjectDetectorNode::goal_result_callback, this, _1);
    options.goal_response_callback = std::bind(&ObjectDetectorNode::goal_response_callback, this, _1);

    // Send the goal
    objects_pub_->async_send_goal(goal, options);
    toDelete = true;
}

void ObjectDetectorNode::goal_result_callback(const SendGoalHandle::WrappedResult &result){
    std::string goal_achieved = result.result->goal_achieved;
    RCLCPP_INFO(this->get_logger(), "%s", goal_achieved.c_str());
}

void ObjectDetectorNode::goal_response_callback(const SendGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal got rejected");
    }

    else {
        RCLCPP_INFO(this->get_logger(), "Goal got accepted");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectorNode>(); 
    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}
