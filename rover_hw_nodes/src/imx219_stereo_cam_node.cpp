#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <ros/ros.h>

// cameras
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS messages
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "imx219_stereo_cam_node");
    ros::NodeHandle n;

    // ImageTransport rather than NodeHandle for better compression, naming conventions, etc for image publishing.
    image_transport::ImageTransport it(n);

    // use CvImagePtr to convert OpenCV-format images to ROS messages
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;

    // advertise our output topics
    image_transport::Publisher im0_pub = it.advertise("/im0", 1);
    image_transport::Publisher im1_pub = it.advertise("/im1", 1);

    // configure our cameras with nvarguscamerasrc pipeline
    cv::VideoCapture cam0("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    cv::VideoCapture cam1("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);

    // make sure both cameras opened correctly
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

    // to hold camera frames
        cv::Mat frame0, frame1;
    sensor_msgs::ImagePtr msg0, msg1;

    // main loop publishes an image from both cameras every iteration
    ros::Rate rate(10); // loop freq. in Hz
    while (ros::ok())
    {
        // write camera streams to frames
        cam0 >> frame0;
        cam1 >> frame1;

        if (!frame0.empty())
        {
            msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0).toImageMsg();
            im0_pub.publish(msg0);
        }

        if (!frame1.empty())
        {
            msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
            im1_pub.publish(msg1);
        }

        // cv::namedWindow("imgLeft", cv::WINDOW_AUTOSIZE);
        // cv::namedWindow("imgRight", cv::WINDOW_AUTOSIZE);
        // cv::imshow("imgLeft", frame0);
        // cv::imshow("imgRight", frame1);

        cv::waitKey(1);

        ros::spinOnce();
        rate.sleep();
    }
}
