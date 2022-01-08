#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS messages
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    // ImageTransport rather than NodeHandle for better compression, naming conventions, etc.
    image_transport::ImageTransport it(n);

    // use CvImagePtr to convert OpenCV-format images to ROS messages
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;

    // advertise our output topics
    image_transport::Publisher im0_pub = it.advertise("/im0", 1);
    image_transport::Publisher im1_pub = it.advertise("/im1", 1);

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
        cv::Mat frame0, frame1, frame2;
        cam2 >> frame2;

        // write camera streams to frames
        cam0 >> frame0;
        cam1 >> frame1;

        // cv_ptr1 = cv)_

        im0_pub.publish(cv_ptr_->toImageMsg());
        im1_pub.publish(cv_ptr_->toImageMsg());

        ros::spinOnce();
        rate.sleep();
    }
}
