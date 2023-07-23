#pragma once 

#include <memory>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rm_wine/yolov7_kpt.hpp"


namespace rm_wine{

yolo_kpt DEMO;
std::vector<yolo_kpt::Object> result;
cv::TickMeter meter;
const double PI =3.14159265;
class Wine :public rclcpp::Node
{
public:
    Wine(const rclcpp::NodeOptions & options);
    ~Wine();
    
private:
    // static void signalHandler(int signum);
    // static bool running;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
    cv::Point2f R_point;//R中心点
    std::vector<cv::Point2f> fun_direction; // 创建 Point2f 类型的向量

    //Camera params
    cv::Mat CAMERA_MATRIX = cv::Mat::eye(3, 3, CV_64F);  // 相机矩阵
    cv::Mat DISTORTION_COEFF = cv::Mat::zeros(1, 5, CV_64F);  // 畸变系数矩阵

    float yaw;  
    float pitch; 
};
}


