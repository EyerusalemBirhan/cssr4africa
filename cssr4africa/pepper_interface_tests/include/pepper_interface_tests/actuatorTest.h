#ifndef ACTUATORTEST_H
#define ACTUATORTEST_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <thread>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>    

#define ROS

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

using namespace boost::algorithm;

// Base class for all the actuators
class Actuator{        
    public:
        std::string actuatorName;
        ControlClientPtr client;
        std::vector<std::string> jointNames;
        std::vector<double> homePosition;
        std::vector<double> maxPosition;
        std::vector<double> minPosition;
        std::vector<std::vector<double>> velocity;
        std::vector<std::vector<double>> duration;
        std::vector<std::vector<double>> positions;
        std::vector<std::string> positionName;

        Actuator(std::string& actuatorName);
        void setTrajectory(const std::vector<std::string>& jointNames, double duration, 
                            std::vector<double> positions);
        std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, 
                            std::vector<std::vector<double>> velocity);

        void testJoint(ControlClientPtr& client, const std::vector<std::string>& jointNames, std::vector<std::vector<double>> duration, 
                            std::vector <double> maxPosition, std::vector<double> minPosition, 
                            std::vector <double> homePosition ,std::vector<std::vector<double>> velocity);

};

class HeadActuator : public Actuator{
    public:
        HeadActuator(std::string actuatorName);       
};

class ArmActuator : public Actuator{
    public:
        ArmActuator(std::string actuatorName);
};

class HandActuator : public Actuator{
    public:
        HandActuator(std::string actuatorName);
};

class LegActuator : public Actuator{
    public:
        LegActuator(std::string actuatorName);
};

class WheelActuator{
    protected:
        ros::Publisher pub;
        geometry_msgs::Twist msg; 
    public:
        WheelActuator(ros::NodeHandle& nh, std::string actuatorName);
        void testWheel();
        void publishVelocity(ros::Publisher& velocityPublisher, geometry_msgs::Twist& msg, ros::Rate& rate, double duration);
};

class configuration {    
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


// Main application class
class ActuatorTestApplication {
    protected:
        ros::NodeHandle nh;
        ControlClientPtr client;

        void testHead(const std::string& name) {
            HeadActuator head(name);
            head.testJoint(client, head.jointNames, head.duration, head.maxPosition, head.minPosition, head.homePosition, head.velocity);
        }

        void testArm(const std::string& name) {
            ArmActuator arm(name);
            arm.testJoint(client, arm.jointNames, arm.duration, arm.maxPosition, arm.minPosition, arm.homePosition, arm.velocity);
        }

        void testHand(const std::string& name) {
            HandActuator hand(name);
            hand.testJoint(client, hand.jointNames, hand.duration, hand.maxPosition, hand.minPosition, hand.homePosition, hand.velocity);
        }

        void testLeg(const std::string& name) {
            LegActuator leg(name);
            leg.testJoint(client, leg.jointNames, leg.duration, leg.maxPosition, leg.minPosition, leg.homePosition, leg.velocity);
        }

        void testWheel(const std::string& name) {
            WheelActuator wheel(nh, name);
            wheel.testWheel();
        }


    public:
        void runTest();
        static std::shared_ptr<Actuator> createActuator(ros::NodeHandle& nh, const std::string& name);
};


#endif // ACTUATORTEST_H