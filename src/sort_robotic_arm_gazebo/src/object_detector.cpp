#include "object_detector.hpp"

ObjectDetectorNode::ObjectDetectorNode() : Node("object_detector_node") {  
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ObjectDetectorNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "creating image publisher");

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>
        ("processed_image", 10);
    RCLCPP_INFO(this->get_logger(), "creating Coordinates publisher");

    objects_pub_ = this->create_publisher<sort_robotic_arm_interface::msg::Coordinates>
        ("detected_objects", 10);
    RCLCPP_INFO(this->get_logger(), "Created the node");

}

void ObjectDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr raw_frame) {
    //RCLCPP_INFO(this->get_logger(), "Starting object detection");
    bool foundObjects = detectObjects(raw_frame);
    if (foundObjects){
        SendObjects();
        //RCLCPP_INFO(this->get_logger(), "After send waiting for response");
        WaitForResponse();
    }
    else{
        WaitOneSecond();
    }
}

bool ObjectDetectorNode::detectObjects(const sensor_msgs::msg::Image::SharedPtr raw_frame) {
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_frame, "bgr8");
        cv::Mat frame = cv_ptr->image;
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        //RCLCPP_INFO(this->get_logger(), "Detecting circles");
        DetectCircles(frame, gray_frame);
        //RCLCPP_INFO(this->get_logger(), "Detecting cubes");
        DetectCubes(frame, gray_frame);

        //RCLCPP_INFO(this->get_logger(), "Publishing image");
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

void ObjectDetectorNode::SendObjects(){
    std::this_thread::sleep_for(std::chrono::seconds(10));
    static bool toDelete = false;
    auto message = sort_robotic_arm_interface::msg::Coordinates();
    if (toDelete) return;
    if ((AllCircles.size() > 0)){
        message.x = AllCircles[0].first;
        message.y = AllCircles[0].second;
    }
    else{ // AllSquars.size() > 0
        message.x = AllSquares[0].first;
        message.y = AllSquares[0].second;
    }
    objects_pub_->publish(message);
    toDelete = true;
}

void ObjectDetectorNode::WaitOneSecond(){
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void ObjectDetectorNode::WaitForResponse(){

}
