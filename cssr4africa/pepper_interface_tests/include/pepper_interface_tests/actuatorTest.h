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

std::string extractTopic(std::string key);
std::string extractMode();
std::vector<std::string> extractTests(std::string set);
void promptAndExit(int status);
void promptAndContinue();
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                    const std::string& positionName, std::vector<double> positions);

std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity);

void head(ros::NodeHandle& nh, std::string head_controller);
void rArm(ros::NodeHandle& nh, std::string rightArm_controller);
void lArm(ros::NodeHandle& nh, std::string leftArm_controller);
void rHand(ros::NodeHandle& nh, std::string rightHand_controller);
void lHand(ros::NodeHandle& nh, std::string leftHand_controller);
void leg(ros::NodeHandle& nh, std::string leg_controller);
void wheels(ros::NodeHandle& nh, std::string wheels_controller);

using namespace boost::algorithm;

#endif // ACTUATORTEST_H