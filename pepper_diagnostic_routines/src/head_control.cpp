#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <thread>

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

void LArmControl(ros::NodeHandle nh, const std::string& controller_name) {
    ControlClientPtr leftArmClient = createClient(controller_name);
    std::vector<std::string> joint_names = {"LElbowRoll", "LElbowYaw", "LShoulderPitch", "LShoulderRoll", "LWristYaw"};
    std::vector<double> positions = {0.5, 0.4, 0.3, 0.10, -1.17};
    double duration = 2.0;
    moveHeadToPosition(leftArmClient, joint_names, duration, "home", positions);
}

void LHand(ros::NodeHandle nh, const std::string& controller_name) {
    ControlClientPtr leftHandClient = createClient(controller_name);
    std::vector<std::string> joint_names = {"LHand"};  
    std::vector<double> positions = {1};
    double duration = 1.0;
    moveHeadToPosition(leftHandClient, joint_names, duration, "home", positions);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "left_arm_control");
    ROS_INFO("Start running left arm control application ...");
    ros::NodeHandle nh;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }

    std::string LeftArm_controller = "/pepper_dcm/LeftArm_controller/follow_joint_trajectory";
    std::string LeftHand_controller = "/pepper_dcm/LeftHand_controller/follow_joint_trajectory";

    std::thread LeftArm_thread(LArmControl, nh, LeftArm_controller);
    std::thread LeftHand_thread(LHand, nh, LeftHand_controller);

    LeftArm_thread.join();
    LeftHand_thread.join();
    
    return 0;
}
