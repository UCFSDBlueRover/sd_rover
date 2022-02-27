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

// integrated IMU
extern "C"
{
    #include "ICM20948.h"
}
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

void buildImuMsg(IMU_ST_ANGLES_DATA stAngles, IMU_ST_SENSOR_DATA stGyroRawData, 
                 IMU_ST_SENSOR_DATA stAccelRawData, IMU_ST_SENSOR_DATA stMagnRawData,
                 sensor_msgs::Imu *imuMsg)
{
    geometry_msgs::Quaternion quatMsg;
    geometry_msgs::Vector3 angularVel;
    geometry_msgs::Vector3 linearAccel;

    // populate header
    imuMsg->header = std_msgs::Header();

    // orientation: convert RPY from degrees to radians
    tf2::Quaternion quat;
    float rollRad = (M_PI / 180) * stAngles.fRoll;
    float pitchRad = (M_PI / 180) * stAngles.fPitch;
    float yawRad = (M_PI / 180) * stAngles.fYaw;

    // create quaternion from RPY
    quat.setRPY(rollRad, pitchRad, yawRad);
    quat.normalize();

    // create a geometry_msgs/Quaternion
    quatMsg = tf2::toMsg(quat);

    // populate orientation with quaternion msg
    // TODO: these values are obviously not based on anything, need to estimate/derive values
    imuMsg->orientation = quatMsg;
    boost::array<double, 9> orientationCovariance = { 1.,0.,0.,
                                                      0.,1.,0.,
                                                      0.,0.,1.};
    imuMsg->orientation_covariance = orientationCovariance;

    // angular velocity from gyroscopes
    angularVel = geometry_msgs::Vector3();
    angularVel.x = stGyroRawData.fX;
    angularVel.y = stGyroRawData.fY;
    angularVel.z = stGyroRawData.fZ;
    imuMsg->angular_velocity = angularVel;

    // populate angular velocity covariance

    boost::array<double, 9> angularVelCovariance = { 1.,0.,0.,
                                                     0.,1.,0.,
                                                     0.,0.,1.};
    imuMsg->angular_velocity_covariance = angularVelCovariance;

    // linear accel from accelerometers
    linearAccel = geometry_msgs::Vector3();
    linearAccel.x = stAccelRawData.fX;
    linearAccel.y = stAccelRawData.fY;
    linearAccel.z = stAccelRawData.fZ;
    imuMsg->linear_acceleration = linearAccel;

    // populate linear accel covariance
    // TODO: these values are obviously not based on anything, need to estimate/derive values
    boost::array<double, 9> linearAccelCovariance = { 1.,0.,0.,
                                                      0.,1.,0.,
                                                      0.,0.,1.};
    imuMsg->linear_acceleration_covariance = linearAccelCovariance;

}


int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "imx219_stereo_cam_node");
    ros::NodeHandle n;

    // ImageTransport rather than NodeHandle for better compression, naming conventions, etc for image publishing.
    image_transport::ImageTransport it(n);

    // advertise our output topics
    image_transport::Publisher im0_pub = it.advertise("/im0", 1);
    image_transport::Publisher im1_pub = it.advertise("/im1", 1);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 10);

    // configure the IMU fields
    IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;
    
    // initialize IMU data connection
    imuInit(&enMotionSensorType);
    if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		printf("Motion sersor is ICM-20948\n" );
	}
	else
	{
		printf("Motion sersor NULL\n");
	}

    // configure our cameras with nvarguscamerasrc (CSI connector) pipeline
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

    // to hold IMU values
    sensor_msgs::Imu imuMsg;

    // main loop publishes an image from both cameras and an IMU message every iteration
    ros::Rate rate(10); // loop freq. in Hz
    while (ros::ok())
    {
        // write camera streams to frames
        cam0 >> frame0;
        cam1 >> frame1;

        // publish camera frames as images (if they're not empty)
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
        cv::waitKey(1);

        // read IMU data
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        
        // populate IMU message
        imuMsg = sensor_msgs::Imu();
        buildImuMsg(stAngles, stGyroRawData, stAccelRawData, stMagnRawData, &imuMsg);

        // publish IMU message
        imu_pub.publish(imuMsg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
