#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

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

void setTrajectoryGoal(control_msgs::FollowJointTrajectoryGoal& goal, const std::vector<std::string>& joint_names,
                       const std::vector<double>& positions, double duration) {
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = positions;
    goal.trajectory.points[0].velocities.resize(positions.size(), 0.05); // Set velocities
    goal.trajectory.points[0].time_from_start = ros::Duration(duration);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "left_arm_control");
    ROS_INFO("Start running left arm control application ...");
    ros::NodeHandle nh;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE;
    }

    ControlClientPtr leftArmClient;
    createClient(leftArmClient, "/pepper_dcm/LeftArm_controller/follow_joint_trajectory");

    control_msgs::FollowJointTrajectoryGoal leftArmGoal;
    setTrajectoryGoal(leftArmGoal, {"LElbowRoll", "LElbowYaw", "LShoulderPitch", "LShoulderRoll", "LWristYaw"},
                      {0.5, 0.4, 0.3, 0.10, -1.17}, 2.0);

    leftArmGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    leftArmClient->sendGoal(leftArmGoal);

    while (!(leftArmClient->getState().isDone()) && ros::ok()) {
        ros::Duration(4.0).sleep();
    }

    return 0;
}
