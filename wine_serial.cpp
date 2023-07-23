#include <iostream>
#include "rm_wine/wine_serial.hpp"


namespace rm_wine{

Wine::Wine(const rclcpp::NodeOptions & options) : Node("rm_wine_serial", options) {
    RCLCPP_INFO(this->get_logger(), "wine is starting");

    // std::signal(SIGINT, &Wine::signalHandler);

    // 订阅原始图像消息
    auto callback = std::bind(&Wine::imageCallback, this, std::placeholders::_1);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/virtual/raw_img", 10, callback);
}

void Wine::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 将ROS图像消息转换为OpenCV图像
    cv_bridge::CvImagePtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "can not transport image: %s", e.what());
        return;
    }

    // 图像处理
    meter.start(); // 计时开始
    result = DEMO.work(image_ptr->image);
    //------
    cv::Point2f fun_point;//识别点
    for (const yolo_kpt::Object& object : result) {
        if (object.label == 0 || object.label == 2)
            { // 只输出标签为0和2的位置信息

              float x0 = object.rect.x;
              float y0 = object.rect.x;
              float x1 = x0 + object.rect.width;
              float y1 = y0 + object.rect.height;

              cv::Point2f keypoints_center(0, 0);
              std::vector<bool> valid_keypoints(5, false);
              for (int i = 0; i < object.kpt.size(); i++) 
              {
              
                if (i != 2 && object.kpt[i].x != 0 && object.kpt[i].y != 0) {
                  valid_keypoints[i] = true;
                }
              }

                // 四种情况判断
                if (valid_keypoints[0] && valid_keypoints[1] && valid_keypoints[3] && valid_keypoints[4]) {
                // 1. 四个关键点都有效，直接取中心点
               keypoints_center = (object.kpt[0] + object.kpt[1] + object.kpt[3] + object.kpt[4]) * 0.25;
                } else if (valid_keypoints[0] && valid_keypoints[3] && (!valid_keypoints[1] || !valid_keypoints[4])) {
               // 2. 0 3关键点有效，1 4 关键点缺少一个以上： 算 0 3 关键点的中点
               keypoints_center = (object.kpt[0] + object.kpt[3]) * 0.5;
                } else if (valid_keypoints[1] && valid_keypoints[4] && (!valid_keypoints[0] || !valid_keypoints[3])) {
                // 3. 1 4关键点有效，0 3 关键点缺少一个以上： 算 1 4 关键点的中点
               keypoints_center = (object.kpt[1] + object.kpt[4]) * 0.5;
                } else {
               // 4. 以上三个都不满足，算bbox中心点
              keypoints_center = cv::Point2f(x0 + object.rect.width / 2, y0 +object.rect.height / 2);
                }
                //std::cout<<"center"<<keypoints_center<<std::endl;
                cv::circle(src_img, keypoints_center, 10, cv::Scalar(0, 0, 255), -1);
                R_point = object.kpt[2];
                fun_point = keypoints_center;
            }
    }
  //std::cout<<"R"<<R_point<<std::endl;
  //std::cout<<"fun"<<fun_point<<std::endl;

//-------预测部分
    //判断方向
    cv::Point nextpoint;//预测点
//--------方向

    fun_direction.push_back(fun_point);
    if(fun_direction.size()>50){
      fun_direction.erase(fun_direction.begin());
    }
    if (fun_direction.size()==50)
    {
      cv::Point2f p1 = fun_direction[0];
      cv::Point2f p2 = fun_direction[20];
      cv::Point2f p3 = fun_direction[40];
      float value = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
      if (value > 0&&num<10) {            
                num++;
            } else if (value < 0&&num<10) {
                num--;
            } 
    }
    if (num>0)
    {
      std::cout << "顺时针" << std::endl;
      small_direction = 10;
      big_direction =1;
    }
    else if(num<0)
    {
      std::cout << "逆时针" << std::endl;
      small_direction = -10;
      big_direction=0;
    }

    //--方向完成

//-----小预测完成
    double radius=sqrt(pow(fun_point.x - R_point.x, 2) + pow(fun_point.y - R_point.y, 2));
    double angle = small_direction * PI/180; //转弧度
    double dx = fun_point.x - R_point.x;
    double dy = fun_point.y - R_point.y;

    nextpoint.x = cos(angle) * dx - sin(angle) * dy + R_point.x;
    nextpoint.y = sin(angle) * dx + cos(angle) * dy + R_point.y;

    std::cout<<"nextpoint "<<nextpoint<<std::endl;

   	//cv::circle(src_img, nextpoint, 10, cv::Scalar(255, 0, 255), -1);
	//cv::imshow("src",src_img);
	  // cv::waitKey(1);

//-------相机结算

    // 设置相机矩阵
    CAMERA_MATRIX.at<double>(0, 0) = 1807.12121;   // fx
    CAMERA_MATRIX.at<double>(1, 1) = 1806.46896;   // fy
    CAMERA_MATRIX.at<double>(0, 2) = 711.11997;    // cx
    CAMERA_MATRIX.at<double>(1, 2) = 562.49495;    // cy

// 设置畸变系数矩阵
    DISTORTION_COEFF.at<double>(0, 0) = -0.078049;   // k1
    DISTORTION_COEFF.at<double>(0, 1) = 0.158627;    // k2
    DISTORTION_COEFF.at<double>(0, 2) = 0.000304;    // p1
    DISTORTION_COEFF.at<double>(0, 3) = -0.000566;   // p2
    DISTORTION_COEFF.at<double>(0, 4) = 0.000000;    // k3

    double fx = CAMERA_MATRIX.at<double>(0, 0);
    double fy = CAMERA_MATRIX.at<double>(1, 1);
    double cx = CAMERA_MATRIX.at<double>(0, 2);
    double cy = CAMERA_MATRIX.at<double>(1, 2);
    cv::Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(nextpoint);
    //对像素点去畸变
    undistortPoints(in, out, CAMERA_MATRIX, DISTORTION_COEFF, cv::noArray(), CAMERA_MATRIX);
    pnt = out.front();
    
    yaw = atan2((pnt.x - cx) ,fx) / CV_PI * 180; //+ (float)(offset_yaw);
    pitch = -atan2((pnt.y - cy) , fy ) / CV_PI * 180; //+ (float)(offset_pitch);

    std::cout<<"yaw "<<yaw<<endl<<"pitch "<<pitch<<endl;

//-----------


    //-----
    meter.stop(); // 计时结束
    RCLCPP_INFO(this->get_logger(), "Time: %f\n", meter.getTimeMilli());
    meter.reset();
}
Wine::~Wine(){}

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<rm_wine::Wine>(options));
    rclcpp::shutdown();
    return 0;
}