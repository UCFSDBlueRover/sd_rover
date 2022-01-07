#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    // advertise our output topics
    ros::Publisher im0_pub = n.advertise<sensor_msgs::Image>("/im0", 10);
    ros::Publisher im1_pub = n.advertise<sensor_msgs::Image>("/im1", 10);

    cv::VideoCapture cam0("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    cv::VideoCapture cam1("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12cv, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);

    if (!cam0.isOpened())
    {
       printf("cam0 is not opened.\n");
       return -1;
    }
    if (!cam1.isOpened())
    {
       printf("cam1 is not opened.\n");
       return -1;
    }


    ros::Rate rate(10);
    while (ros::ok())
    {
	
        ros::spinOnce();
        rate.sleep();
    }
}
