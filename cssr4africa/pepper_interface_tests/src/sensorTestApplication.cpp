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
    // Get the tests to run
    std::vector<std::string> testNames;
    
    char start_buf[50];
    
    std::time_t start_t = std::time(0);
    std::tm* start_now = std::localtime(&start_t);
    strftime(start_buf, sizeof(start_buf), "%Y-%m-%d.%X", start_now);

    testNames = extractTests("sensor");

    string path;
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        path = "..";
    #endif
    
    // complete the path of the output file
    path += "/data/sensorTestOutput.dat";
    
    std::ofstream out_of;
    out_of.open(path.c_str(), ofstream::app);
    if (!out_of.is_open()){
        printf("Unable to open the output file %s\n", path.c_str());
        promptAndExit(1);
    }
    out_of << "[TESTING] ############ SENSORS ############\n\n";
    out_of << "[START TIME] " << start_buf << "\n";
    
    out_of.close();
    
    // Initialize the ROS node
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

    // Get the mode
    std::string mode = extractMode();
    
    if (mode == "parallel"){
        // Run each test in a separate thread
        std::vector<std::thread> threads;
        for (auto test : testNames){
            if (test == "backsonar"){
                threads.push_back(std::thread(backSonar, nh));
            }
            else if (test == "frontsonar"){
                threads.push_back(std::thread(frontSonar, nh));
            }
            else if (test == "frontcamera"){
                threads.push_back(std::thread(frontCamera, nh));
            }
            else if (test == "bottomcamera"){
                threads.push_back(std::thread(bottomCamera, nh));
            }
            else if (test == "depthcamera"){
                threads.push_back(std::thread(depthCamera, nh));
            }
            else if (test == "lasersensor"){
                threads.push_back(std::thread(laserSensor, nh));
            }
            else if (test == "stereocamera"){
                threads.push_back(std::thread(stereoCamera, nh));
            }
            else{
                std::cout << "No test provided. Exiting...\n";
                promptAndExit(1);
            }
        }
        // Wait for all threads to finish
        for (auto& th : threads){
            th.join();
        }
    }
    else if (mode == "sequential"){
        // Run each test sequentially
        for (auto test : testNames){
            if (test == "backsonar"){
                backSonar(nh);
            }
            else if (test == "frontsonar"){
                frontSonar(nh);
            }
            else if (test == "frontcamera"){
                frontCamera(nh);
            }
            else if (test == "bottomcamera"){
                bottomCamera(nh);
            }
            else if (test == "depthcamera"){
                depthCamera(nh);
            }
            else if (test == "lasersensor"){
                laserSensor(nh);
            }
            else if (test == "stereocamera"){
                stereoCamera(nh);
            }
            else{
                std::cout << "No test provided. Exiting...\n";
                promptAndExit(1);
            }
        }
    }
    else{
        std::cout << "No mode provided. Exiting...\n";
        promptAndExit(1);
    }

    char end_buf[50];
    
    std::time_t end_t = std::time(0);
    std::tm* end_now = std::localtime(&end_t);
    strftime(end_buf, sizeof(end_buf), "%Y-%m-%d.%X", end_now);

    out_of.open(path.c_str(), ofstream::app);    
    out_of << "[END TIME] " << end_buf << "\n\n";
    out_of.close();

    return 0;
}