# include "pepper_interface_tests/actuatorTest.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "actuatorTest");
    ros::NodeHandle nh;

    // Create the actuator object
    ActuatorTestApplication app;

    // Run the application
    app.runTest();
          
    return 0;
}
