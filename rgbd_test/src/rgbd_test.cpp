#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int MIN_H = 18;
int MIN_S = 138;
int MIN_V = 116;
int MAX_H = 44;
int MAX_S = 190;
int MAX_V = 243;

static const string RGB_Window = "RGB Image";
static const string Gray_Window = "Gray Image";
static const string HSV_Window = "HSV Image";
static const string Thresholded_Window = "Thresholded Image";
static const string Track_Window = "Track Image";
//定义一个转换的类
class KINECT2_ROS
{
  private:
    ros::NodeHandle nh_; //定义ROS句柄
    image_transport::ImageTransport it_; //定义一个image_transport实例
    image_transport::Subscriber hsv_sub_;//hsv
    image_transport::Publisher image_pub_; //定义ROS图象发布器
  public:
    Mat rgbImage;
    KINECT2_ROS()
      :it_(nh_) //构造函数
    {
        hsv_sub_ = it_.subscribe("/kinect2/sd/image_color_rect", 1,&KINECT2_ROS::img_callback, this);
        image_pub_ = it_.advertise("/kinect2/sd/image_color/output", 1);
        //初始化输入输出窗口
        namedWindow("Trackbars");
        createTrackbar("MIN_H","Trackbars",&MIN_H,180,NULL);
        createTrackbar("MAX_H","Trackbars",&MAX_H,180,NULL);
        createTrackbar("MIN_S","Trackbars",&MIN_S,256,NULL);
        createTrackbar("MAX_S","Trackbars",&MAX_S,256,NULL);
        createTrackbar("MIN_V","Trackbars",&MIN_V,256,NULL);
        createTrackbar("MAX_V","Trackbars",&MAX_V,256,NULL);

        namedWindow(RGB_Window);
        namedWindow(Gray_Window);
        namedWindow(HSV_Window);
        namedWindow(Thresholded_Window);
        namedWindow(Track_Window);
    }

    ~KINECT2_ROS() //析构函数
    {
         destroyWindow(RGB_Window);
         destroyWindow(Gray_Window);
         destroyWindow(HSV_Window);
         destroyWindow(Thresholded_Window);//Track_Window
         destroyWindow(Track_Window);
    }

    void img_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(rgbImage);

        Mat image_gray,img_hsv,img_mask;

        cvtColor(cv_ptr->image, image_gray, CV_RGB2GRAY);//灰度处理
        cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);//hsv处理

        inRange(img_hsv,cv::Scalar(MIN_H,MIN_S,MIN_V),cv::Scalar(MAX_H,MAX_S,MAX_V),img_mask);//滚动条范围

        imshow(RGB_Window, cv_ptr->image);
        imshow(Gray_Window, image_gray);
        imshow(HSV_Window, img_hsv);
        imshow(Thresholded_Window, img_mask);

        Mat result = rgbImage.clone();
        //查找轮廓并绘制轮廓
        vector<vector<Point> > contours;
        findContours(img_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        drawContours(result, contours, -1, Scalar(0, 0, 255), 2);//在result上绘制轮廓
        //查找正外接矩形
        vector<Rect> boundRect(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            boundRect[i] = boundingRect(contours[i]);
            rectangle(result, boundRect[i], Scalar(0, 255, 0), 2);//在result上绘制正外接矩形
        }
        imshow(Track_Window, result);

        waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Image_test");
    KINECT2_ROS obj;

    ros::spin();
}
