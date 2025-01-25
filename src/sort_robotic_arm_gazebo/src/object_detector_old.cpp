#include "object_detector.hpp"

ObjectDetectorNode::ObjectDetectorNode() : Node("object_detector_node") {  

    last_execution_time_ = std::chrono::steady_clock::now();

    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ObjectDetectorNode::imageCallback, this, std::placeholders::_1));
        
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>
        ("processed_image", 10);
    
    objects_pub_ = this->create_publisher<sort_robotic_arm_interface::msg::CoordinatesList>
        ("detected_objects", 10);

}

void ObjectDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr raw_frame) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - last_execution_time_).count();

    if (elapsed_time >= 1) { // Check if 1 second has passed
        last_execution_time_ = now; // Update the last execution time
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_frame, "bgr8");
            cv::Mat frame = cv_ptr->image;

            cv::Mat gray_frame;
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            detectObjects(frame, gray_frame, AllCircles, AllSquares);
            SendObjects();

            sensor_msgs::msg::Image::SharedPtr out_msg = 
               cv_bridge::CvImage(raw_frame->header, "bgr8", frame).toImageMsg();
            image_pub_->publish(*out_msg);
        }
        catch(cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
}


void ObjectDetectorNode::detectObjects(cv::Mat& frame, cv::Mat& gray_frame,  std::vector<std::pair<int, int>> &AllCircles, std::vector<std::pair<int, int>> &AllSquars) {
    // Detect circles (balls)
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray_frame, circles, cv::HOUGH_GRADIENT, 1,
                     gray_frame.rows / 8, 100, 20, 0.001, 100);
                    
    for(const auto& circle : circles) {
        std::pair<int, int> circleCoor = {cvRound(circle[0]), cvRound(circle[1])};
        AllCircles.push_back(circleCoor);
        processCircle(circle, frame);
    }

    // Detect cubes
    cv::Mat edges;
    cv::Canny(gray_frame, edges, 10, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for(const auto& contour : contours) {
        std::pair<int, int> squareCoor = CheckForCube(contour);
        if (squareCoor.first != -1){ // -1 says its not a cube
            processCube(contour, frame);
            AllSquars.push_back(squareCoor);
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
                // RCLCPP_WARN(this->get_logger(), "Rect Width: %d", rect.width);
                return {x_center, y_center};
            }
        }
    }
    return {-1,-1};
}

void ObjectDetectorNode::SendObjects(){
    auto message = sort_robotic_arm_interface::msg::CoordinatesList();

    // Add circles to the message
    // for (const auto& circle : AllCircles) {
    //     sort_robotic_arm_interface::msg::Coordinates coord;
    //     coord.x = circle.first;
    //     coord.y = circle.second;
    //     message.points.push_back(coord);
    // }

    // Add squares to the message
    for (const auto& square : AllSquares) {
        sort_robotic_arm_interface::msg::Coordinates coord;
        coord.x = square.first;
        coord.y = square.second;
        message.points.push_back(coord);
    }

    objects_pub_->publish(message);
}