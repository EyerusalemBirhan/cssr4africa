// #include <ros/ros.h>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <vector>
// #include <string>
// #include <boost/shared_ptr.hpp>

// typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
// typedef boost::shared_ptr<ControlClient> ControlClientPtr;

// ControlClientPtr createClient(const std::string& controller_name) {
//     ControlClientPtr actionClient(new ControlClient(controller_name, true));
//     int max_iterations = 5;

//     for (int iterations = 0; iterations < max_iterations; ++iterations) {
//         if (actionClient->waitForServer(ros::Duration(5.0))) {
//             return actionClient;
//         }
//         ROS_DEBUG("Waiting for the %s controller to come up", controller_name.c_str());
//     }

//     throw std::runtime_error("Error creating action client for " + controller_name + " controller: Server not available");
// }

// void moveHeadToPosition(ControlClientPtr& headClient, const std::string& joint_name, double position, double duration, const std::string& position_name) {
//     control_msgs::FollowJointTrajectoryGoal goal;
//     trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
//     trajectory.joint_names = {joint_name};
//     trajectory.points.resize(1);
    
//     trajectory.points[0].positions = {0.0};
//     trajectory.points[0].time_from_start = ros::Duration(duration);

//     goal.goal_time_tolerance = ros::Duration(0.0);
//     headClient->sendGoal(goal);
//     bool success = headClient->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed

//     if (success) {
//          ROS_INFO_STREAM("Moved " << joint_name << " to position: " << position_name << " (" << position << ")");

//     } else {
//         ROS_ERROR_STREAM("Failed to move " << joint_name);
//     }
// }

// void headControl(ros::NodeHandle nh) {
//     ControlClientPtr headClient = createClient("/pepper_dcm/Head_controller/follow_joint_trajectory");

//     std::vector<std::string> joint_names = {"HeadYaw", "HeadPitch"};
//     std::vector<double> max_positions = {2.0857, 0.4451};
//     std::vector<double> min_positions = {-2.0857, -0.7068};

//     // mapping joint name to index 
//     std::map<std::string, int> joint_index;
//     joint_index["HeadYaw"] = 0;
//     joint_index["HeadPitch"] = 1;


//     double duration = 2.0;

//     ROS_INFO("----------[START HEAD CONTROL TEST]-----------");

//     for (const std::string& joint : joint_names) {
//         ROS_INFO_STREAM("[START] " << joint << " test.");

//         // Move to maximum position
//         moveHeadToPosition(headClient, joint, max_positions[joint_index[joint]], duration, "maximum position");
        
//         // Move to minimum position
//         moveHeadToPosition(headClient, joint, min_positions[joint_index[joint]], duration, "minimum position");

//         // // Move to mid position
//         double mid_position = (max_positions[joint_index[joint]] + min_positions[joint_index[joint]]) / 2;
//         moveHeadToPosition(headClient, joint, mid_position, duration, "mid position");

//         ROS_INFO_STREAM("[END] " << joint << " test.");
//     }

//     // End of the program
//     ROS_INFO_STREAM("[SUCCESS] Head control test completed.");
//     ROS_INFO_STREAM("                                      ");
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "head_control_test");
//     ros::NodeHandle nh;
//     headControl(nh);

//     return 0;
// }

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

ControlClientPtr createClient(const std::string& controller_name) {
    ControlClientPtr actionClient(new ControlClient(controller_name, true));
    int max_iterations = 5;

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", controller_name.c_str());
    }

    throw std::runtime_error("Error creating action client for " + controller_name + " controller: Server not available");
}

void createClient(ControlClientPtr& actionClient, const std::string& controller_name) {
    ROS_INFO("Creating action client for %s controller ...", controller_name.c_str());
    actionClient.reset(new ControlClient(controller_name, true));

    int max_iterations = 5;
    int iterations = 0;

    while (!actionClient->waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations) {
        ROS_DEBUG("Waiting for the %s controller to come up", controller_name.c_str());
        ++iterations;
    }

    if (iterations == max_iterations) {
        throw std::runtime_error("Error creating action client for " + controller_name + " controller: Server not available");
    }
}


void moveHeadToPosition(ControlClientPtr& headClient, const std::vector<std::string>& joint_names, double duration, 
                        const std::string& position_name, std::vector<double> positions) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].velocities.resize(positions.size(), 0.05); // Set velocities
    trajectory.points[0].time_from_start = ros::Duration(duration);

    headClient->sendGoal(goal);
    headClient->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
}

void headControl(ros::NodeHandle nh) {
    ControlClientPtr headClient = createClient("/pepper_dcm/Head_controller/follow_joint_trajectory");

    std::vector<std::string> joint_names = {"HeadPitch", "HeadYaw"};
    std::vector<double> max_positions = {0.4451, 2.0857 };
    std::vector<double> min_positions = {-0.7068, -2.0857};

    // mapping joint name to index 
    std::map<std::string, int> joint_index;
    joint_index["HeadPitch"] = 0;
    joint_index["HeadYaw"] = 1;

    // Determine the number of joints
    int num_joints = joint_names.size();

    // Initialize the positions vector
    std::vector<double> positions(num_joints, 0.0);

    double duration = 1.5;

    ROS_INFO("----------[START HEAD CONTROL TEST]-----------");

    for (const std::string& joint : joint_names) {
        ROS_INFO_STREAM("[START] " << joint << " test.");

        // Move to maximum position
        positions[joint_index[joint]] = max_positions[joint_index[joint]];
        moveHeadToPosition(headClient, joint_names, duration, "maximum position", positions);
        ROS_INFO_STREAM("Moved " << joint << " to position: " << "maximum position" << " (" << max_positions[joint_index[joint]] << ")");
        
        // Move to minimum position
        positions[joint_index[joint]] = min_positions[joint_index[joint]];
        moveHeadToPosition(headClient, joint_names, duration, "minimum position", positions);
        ROS_INFO_STREAM("Moved " << joint << " to position: " << "minimum position" << " (" << min_positions[joint_index[joint]] << ")");


        // Move to mid position
        double mid_position = (max_positions[joint_index[joint]] + min_positions[joint_index[joint]]) / 2;
        positions[joint_index[joint]] = mid_position;
        moveHeadToPosition(headClient, joint_names, duration, "mid position", positions);
        ROS_INFO_STREAM("Moved " << joint << " to position: " << "mid position" << " (" << mid_position << ")");

        ROS_INFO_STREAM("[END] " << joint << " test.");
    }

    // End of the program
    ROS_INFO_STREAM("[SUCCESS] Head control test completed.");
    ROS_INFO_STREAM("                                      ");
}

// LeftArmControl
void LArmControl(ros::NodeHandle nh) {
    ControlClientPtr LArmClient = createClient("/pepper_dcm/LArm_controller/follow_joint_trajectory");

    std::vector<std::string> joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};
    std::vector<double> max_positions = {2.0857, 1.5620,  -0.0087,  2.0857,  1.8239};
    std::vector<double> min_positions = {-2.0857, 0.0087, -1.5620, -2.0857, -1.8239};

    // mapping joint name to index 
    std::map<std::string, int> joint_index;
    joint_index["LShoulderPitch"] = 0;
    joint_index["LShoulderRoll"] = 1;
    joint_index["LElbowYaw"] = 2;
    joint_index["LElbowRoll"] = 3;
    joint_index["LWristYaw"] = 4;

    // Determine the number of joints
    int num_joints = joint_names.size();

    // Initialize the positions vector
    std::vector<double> positions(num_joints, 0.0);

    double duration = 2;

    ROS_INFO("----------[START LArm CONTROL TEST]-----------");

    for (const std::string& joint : joint_names) {
        ROS_INFO_STREAM("[START] " << joint << " test.");

        // Move to maximum position
        positions[joint_index[joint]] = max_positions[joint_index[joint]];
        moveHeadToPosition(LArmClient, joint_names, duration, "maximum position", positions);
        ROS_INFO_STREAM("Moved " << joint << " to position: " << "maximum position" << " (" << max_positions[joint_index[joint]] << ")");
        
        // Move to minimum position
        positions[joint_index[joint]] = min_positions[joint_index[joint]];
        moveHeadToPosition(LArmClient, joint_names, duration, "minimum position", positions);
        ROS_INFO_STREAM("Moved " << joint << " to position: " << "minimum position" << " (" << min_positions[joint_index[joint]] << ")");

        ROS_INFO_STREAM("[END] " << joint << " test .");
    }

    // End of the program
    ROS_INFO_STREAM("[SUCCESS] LArm control test completed.");
    ROS_INFO_STREAM("                                      ");
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "head_control_test");
    ros::NodeHandle nh;
    // headControl(nh);
    // LhandControl(nh);
    LArmControl(nh);

    return 0;
}
