/***************************************************************************************************************************
 * @file sensorTest.cpp
 * @brief Subscribes to the topics of the sensors of the Pepper robot to verify that the sensors reading is being published 
 *        on their corresponding topics. The topics are extracted from the configuration file and the expected tests to run
 *        are extracted from the input file. The output is saved in the output file. The sensor that will be tested are:
 *        BackSonar, FrontSonar, FrontCamera, BottomCamera, DepthCamera, LaserSensor.
 *           
 * @author CSSR4Africa Team
 * @version 1.0
 * @date September 07, 2023
 *  
 ***************************************************************************************************************************/

# include "pepper_interface_tests/sensorTest.h"

/* Main function */
int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

    // Create the sensor objects
    SensorTestApplication app;
    
    // Run the tests
    app.runTests();
}