#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>

# include <ros/ros.h>
# include <ros/package.h>
# include <std_msgs/Float64.h>
# include <sensor_msgs/CameraInfo.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>
# include <thread>

#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

using namespace boost;
using namespace std;

#define ROS

class Sensors{
    public:
        ros::NodeHandle nh;
        std::string sensorName;
        bool output;
        double timeDuration;
        Sensors(ros::NodeHandle nh, const std::string& sensorName);
};

class RgbCamera : public Sensors{
    public:
        RgbCamera(ros::NodeHandle nh, const std::string& sensorName);
        void rgbCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
};

class DepthCamera : public Sensors{
    public:
        DepthCamera(ros::NodeHandle nh, std::string& sensorName);
        void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
};

class LaserSensor : public Sensors{
    public:
        LaserSensor(ros::NodeHandle nh, std::string& sensorName);
        void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg);
};

class SonarSensor : public Sensors{
    public:
        SonarSensor(ros::NodeHandle nh, std::string& sensorName);
        void sonarSensorMessageReceived(const sensor_msgs::Range& msg);
};

class Microphone : public Sensors{
    public:
        Microphone(ros::NodeHandle nh, std::string& sensorName);
        void microphoneMessageReceived(const sensor_msgs::Range& msg);
};

class Configuration {    
    public:
        static std::string extractTopic(std::string key);
        static std::string extractMode();
        static std::vector<std::string> extractTests(std::string set);

};

class Utility{
    public:
        static void promptAndExit(int status);
        static void promptAndContinue();
};

// Main application Test class
class SensorTestApplication{
    protected:
        ros::NodeHandle nh;
        
        void testrgbCamera(const std::string& sensorName){
            RgbCamera rgbCamera(nh, sensorName);
        }
        void testdepthCamera(const std::string& sensorName);
        void testlaserSensor(const std::string& sensorName);
        void testsonarSensor(const std::string& sensorName);
        void testmicrophone(const std::string& sensorName);

    public:
        void runTests();

};

#endif // SENSOR_TEST_H