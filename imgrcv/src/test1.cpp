#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/msg/point32.hpp>

using namespace cv;
using namespace std;

class test1 : public rclcpp::Node
{
public:
    test1() : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/raw_image", 10, std::bind(&test1::imageCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("click_position", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            Mat imgRaw = cv_ptr->image;
            Mat imgDraw = imgRaw.clone();
            Mat imgHSV, maskLine, maskTap;
            //----------------------------------------//
            rectangle(imgRaw,Point(800,100),Point(1140,0),Scalar(0,0,0),FILLED);
            rectangle(imgRaw,Point(0,0),Point(100,50),Scalar(0,0,0),FILLED);
            cvtColor(imgRaw,imgHSV,COLOR_BGR2HSV);
            //------------------Line Processing Begin--------------------//
            Mat lineWhite, lineBlur, lineCanny, lineDilate, lineErode;
            inRange(imgHSV, Scalar(0, 0, 255), Scalar(179, 0, 255), lineWhite);
            GaussianBlur(lineWhite,lineBlur,Size(3,3),0,0);
            Canny(lineBlur,lineCanny,100,100,3);
            dilate(lineCanny,lineDilate,getStructuringElement(MORPH_RECT, Size(5, 5)));
            erode(lineDilate,lineErode,getStructuringElement(MORPH_RECT, Size(5, 5)));
            //imshow("image",lineErode);

            vector<Vec4i> lineDetected;
            HoughLinesP(lineDilate, lineDetected, 1, CV_PI / 180, 40, 10, 10);
            int maxLength = 0, maxLengthi = 0;
            for (size_t i = 0; i < lineDetected.size(); i++) {
              Vec4i l = lineDetected[i];
              double length = sqrt(pow((l[0]-l[2]),2)+pow((l[1]-l[3]),2));
              if(length > maxLength) {
                maxLength = length;
                maxLengthi = i;
              }
            }
            Vec4i l = lineDetected[maxLengthi];
            //cout << "length = " << maxLength << endl;
            double slope = (double)(l[3] - l[1]) / (l[2] - l[0]);
            double intercept = l[1] - slope * l[0];
            //string str1 = "y = " + to_string(slope) + "x + " + to_string(intercept);
            //putText(imgDraw,str1,Point(0,610),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
            line(imgDraw, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);

            //------------------Line Processing End--------------------//
            //------------------Tap Processing Begin--------------------//
            Scalar lowerTap(83, 230, 0);
            Scalar upperTap(102, 255, 255);
            Mat imgBlur, imgCanny, imgDil;
            inRange(imgHSV,lowerTap,upperTap,maskTap);
            GaussianBlur(maskTap, imgBlur, Size(5, 5), 5, 0);
            Canny(imgBlur, imgCanny, 50, 50, 3);
            Mat kernel3 = getStructuringElement(MORPH_RECT, Size(5, 5));
            dilate(imgCanny,imgDil,kernel3);
            //imshow("Canny",imgDil);

            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(imgDil,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

            for(size_t i=0;i<contours.size();i++) {
                Moments M = moments(contours[i]);
                int cX = M.m10/M.m00;
                int cY = M.m01/M.m00;
                //drawContours(imgDraw,contours,i,Scalar(0,255,0),3);
                //circle(imgDraw, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
                //string str2 = " " + to_string(cX) + " " + to_string(cY);
                //putText(imgDraw,str2,Point(cX,cY),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
                //solve targetpoints
                int targetX = (cX/slope+cY-intercept)/(slope+1/slope);
                int targetY = slope*targetX + intercept;
                double distance = sqrt(pow(targetX-cX,2)+pow(targetY-cY,2));
                //line(imgDraw, Point(targetX,targetY), Point(cX,cY), Scalar(255, 255, 255), 1, LINE_AA);
                if(distance<50){
                    //circle(imgDraw, cv::Point(targetX, targetY), 10, cv::Scalar(0, 255 , 0), -1);
                    geometry_msgs::msg::Point32 info;
                    info.x = 1.0f*targetX;
                    info.y = 1.0f*targetY;
                    RCLCPP_INFO(this->get_logger(), "Publishing: (%f, %f)", info.x, info.y);
                    publisher_->publish(info);
                    break;
                }
                //else circle(imgDraw, cv::Point(targetX, targetY), 5, cv::Scalar(0, 255, 255), -1);


                //imshow("1", imgDraw);

            }
            //------------------Tap Processing End--------------------//

            //----------------------------------------//
            waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }



    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::Node> node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test1>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}