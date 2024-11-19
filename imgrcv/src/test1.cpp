#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <oneapi/tbb/task_arena.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
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
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            Mat img = cv_ptr->image;
            Mat imgDraw = cv_ptr->image;
            Mat imgHSV, maskLine, maskTap;
            //----------------------------------------//
            rectangle(img,Point(800,100),Point(1140,1),Scalar(0,0,0),FILLED);
            rectangle(img,Point(0,0),Point(100,50),Scalar(0,0,0),FILLED);
            cvtColor(img,imgHSV,COLOR_BGR2HSV);
            //------------------Line Processing Begin--------------------//
            Scalar lowerLine(0, 0, 255);
            Scalar upperLine(179, 0, 255);
            inRange(imgHSV,lowerLine,upperLine,maskLine);
            Mat dil;
            Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
            dilate(maskLine,dil,kernel);
            Mat kernel2 = getStructuringElement(MORPH_RECT, Size(13, 13));
            erode(dil,dil,kernel2);

            vector<Vec4i> lines;
            HoughLinesP(dil, lines, 1, CV_PI / 180, 40, 10, 10);
            int maxLength = 0, maxLengthi = 0;
            for (size_t i = 0; i < lines.size(); i++) {
              Vec4i l = lines[i];
              double length = sqrt(pow((l[0]-l[2]),2)+pow((l[1]-l[3]),2));
              if(length > maxLength) {
                maxLength = length;
                maxLengthi = i;
              }
            }
            Vec4i l = lines[maxLengthi];
            //cout << "length = " << maxLength << endl;
            double slope = (double)(l[3] - l[1]) / (l[2] - l[0]);
            double intercept = l[1] - slope * l[0];
            //string str1 = "y = " + to_string(slope) + "x + " + to_string(intercept);
            //putText(imgDraw,str1,Point(0,610),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
            //line(imgDraw, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);

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
                if(distance<40){
                    //circle(imgDraw, cv::Point(targetX, targetY), 10, cv::Scalar(0, 255 , 0), -1);
                    geometry_msgs::msg::Point32 info;
                    info.x = 1.0f*targetX; // 示例x坐标
                    info.y = 1.0f*targetY; // 示例y坐标
                    info.z = 0.0f; // 对于Point32，z坐标总是0
                    RCLCPP_INFO(this->get_logger(), "Publishing: (%f, %f, %f)", info.x, info.y, info.z);
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
/*
clickPointSubscription = create_subscription<geometry_msgs::msg::Point32>(
                "/click_position",
                10,
                [this](geometry_msgs::msg::Point32::SharedPtr msg) {
                    try {
                        float info[2]{msg->x, msg->y};
                        server.send(hdl, &info, sizeof(info), websocketpp::frame::opcode::value::binary);
                        RCLCPP_INFO_STREAM(this->get_logger(),
                                           "Send position: (" << msg->x << " " << msg->y << ")");
                    } catch (websocketpp::exception const &e) {
                        RCLCPP_FATAL_STREAM(this->get_logger(),
                                            "Send failed because: " << "(" << e.what() << ")");
                    }
                });
imagePublisher = create_publisher<sensor_msgs::msg::Image>("/raw_image", 10);
*/