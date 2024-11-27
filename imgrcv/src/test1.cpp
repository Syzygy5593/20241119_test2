#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/msg/point32.hpp>

using namespace cv;
using namespace std;

class test1: public rclcpp::Node {
    public: test1(): Node("image_subscriber") {
        subscription_ = this -> create_subscription < sensor_msgs::msg::Image > (
            "/raw_image", 10, std::bind( & test1::imageCallback, this, std::placeholders::_1));
        publisher_ = this -> create_publisher < geometry_msgs::msg::Point32 > ("click_position", 10);
    }

    private: void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            Mat imgRaw = cv_ptr -> image;
            Mat imgDraw = imgRaw.clone();
            Mat imgHSV, maskTap;
            cvtColor(imgRaw, imgHSV, COLOR_BGR2HSV);
            //------------------Line Processing Begin--------------------//
            Mat lineRange, lineBlur, lineCanny, lineDilate, lineErode;
            //预处理
            inRange(imgHSV, Scalar(0, 0, 255), Scalar(179, 0, 255), lineRange);
            GaussianBlur(lineRange, lineBlur, Size(3, 3), 0, 0);
            Canny(lineBlur, lineCanny, 100, 100, 3);
            dilate(lineCanny, lineDilate, getStructuringElement(MORPH_RECT, Size(5, 5)));
            erode(lineDilate, lineErode, getStructuringElement(MORPH_RECT, Size(5, 5)));
            //霍夫检测 提取最长值
            vector < Vec4i > lineDetected;
            HoughLinesP(lineErode, lineDetected, 1, CV_PI / 180, 40, 100, 10);
            int maxLength = 0, maxLengthi = 0;
            for (size_t i = 0; i < lineDetected.size(); i++) {
                Vec4i l = lineDetected[i];
                double length = sqrt(pow((l[0] - l[2]), 2) + pow((l[1] - l[3]), 2));
                if (length > maxLength) {
                    maxLength = length;
                    maxLengthi = i;
                }
            }
            Vec4i l = lineDetected[maxLengthi];
            //求解析式
            double slope = (double)(l[3] - l[1]) / (l[2] - l[0]); //斜率
            double intercept = l[1] - slope * l[0]; //截距
            line(imgDraw, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
            //RCLCPP_INFO(this->get_logger(), "k=%f, b=%f", slope, intercept);
            //------------------Line Processing End--------------------//
            //------------------Tap Processing Begin--------------------//
            Mat tapRange, tapBlur, tapCanny, tapDilate;
            //预处理 直接颜色
            inRange(imgHSV, Scalar(83, 230, 0), Scalar(102, 255, 255), tapRange);
            GaussianBlur(tapRange, tapBlur, Size(5, 5), 5, 0);
            Canny(tapBlur, tapCanny, 50, 50, 3);
            dilate(tapCanny, tapDilate, getStructuringElement(MORPH_RECT, Size(5, 5)));
            //查找边缘，然后定位中心点
            vector < vector < Point >> contours;
            vector < Vec4i > hierarchy;
            findContours(tapDilate, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (size_t i = 0; i < contours.size(); i++) {
                Moments M = moments(contours[i]);
                int pointX = M.m10 / M.m00;
                int pointY = M.m01 / M.m00;
                //求解点击坐标
                int targetX, targetY;
                if (slope < 0.01 && slope > -0.01) {
                    targetX = pointX;
                    targetY = intercept;
                } else {
                    targetX = (pointX / slope + pointY - intercept) / (slope + 1 / slope);
                    targetY = slope * targetX + intercept;
                }
                //接近判定线时发送点击消息
                double distance = sqrt(pow(targetX - pointX, 2) + pow(targetY - pointY, 2));
                if (distance < 50) {
                    geometry_msgs::msg::Point32 info;
                    info.x = 1.0f * targetX;
                    info.y = 1.0f * targetY;
                    RCLCPP_INFO(this -> get_logger(), "Publishing: (%f, %f)", info.x, info.y);
                    publisher_ -> publish(info);
                    break;
                }
            }
            //------------------Tap Processing End--------------------//
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this -> get_logger(), "Could not convert image: %s", e.what());
        }
    }
    rclcpp::Publisher < geometry_msgs::msg::Point32 > ::SharedPtr publisher_;
    rclcpp::Subscription < sensor_msgs::msg::Image > ::SharedPtr subscription_;
    std::shared_ptr < rclcpp::Node > node_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared < test1 > ();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}