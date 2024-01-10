# include "pepper_interface_tests/actuatorTest.h"

int main(int argc, char** argv) {
    std::vector<std::string> testName = extractTests("actuator");

    // Initialize ROS
    ros::init(argc, argv, "actuatorTest");
    ros::NodeHandle nh;

    // Extract the mode to run the tests
    std::string mode = extractMode();
    std::cout << "Mode: " << mode << std::endl;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }
    
    // Run the tests in the mode specified in the configuration file
    if (mode == "sequential"){
        for (int i = 0; i < testName.size(); ++i){
            if (testName[i] == "Head"){
                std::string headTopic = extractTopic("Head");
                head(nh, headTopic);
            }
            else if (testName[i] == "RArm"){
                std::string rightArmTopic = extractTopic("RArm");
                rArm(nh, rightArmTopic);
            }
            else if (testName[i] == "LArm"){
                std::string leftArmTopic = extractTopic("LArm");
                lArm(nh, leftArmTopic);
            }
            else if (testName[i] == "RHand"){
                std::string rightHandTopic = extractTopic("RHand");
                rHand(nh, rightHandTopic);
            }
            else if (testName[i] == "LHand"){
                std::string leftHandTopic = extractTopic("LHand");
                lHand(nh, leftHandTopic);
            }
            else if (testName[i] == "Leg"){
                std::string legTopic = extractTopic("Leg");
                leg(nh, legTopic);
            }
            else if (testName[i] == "Wheels"){
                std::string wheelsController = extractTopic("Wheels");
                wheels(nh);
            }
        }
    }

    else if (mode == "parallel"){
        std::vector<std::thread> threads;
        for (int i = 0; i < testName.size(); ++i){
            if (testName[i] == "Head"){
                std::string headTopic = extractTopic("Head");
                threads.push_back(std::thread(head, std::ref(nh), headTopic));
            }
            else if (testName[i] == "RArm"){
                std::string rightArmTopic = extractTopic("RArm");
                threads.push_back(std::thread(rArm, std::ref(nh), rightArmTopic));
            }
            else if (testName[i] == "LArm"){
                std::string leftArmTopic = extractTopic("LArm");
                threads.push_back(std::thread(lArm, std::ref(nh), leftArmTopic));
            }
            else if (testName[i] == "RHand"){
                std::string rightHandTopic = extractTopic("RHand");
                threads.push_back(std::thread(rHand, std::ref(nh), rightHandTopic));
            }
            else if (testName[i] == "LHand"){
                std::string leftHandTopic = extractTopic("LHand");
                threads.push_back(std::thread(lHand, std::ref(nh), leftHandTopic));
            }
            else if (testName[i] == "Leg"){
                std::string legTopic = extractTopic("Leg");
                threads.push_back(std::thread(leg, std::ref(nh), legTopic));
            }
            else if (testName[i] == "Wheels"){
                std::string wheelsController = extractTopic("Wheels");
                threads.push_back(std::thread(wheels, std::ref(nh)));
            }
        }
        for (auto& th : threads) th.join();
    }
    else{
        printf("Invalid mode. Please check the mode in the configuration file.\n");
        promptAndExit(1);
    }
          
    return 0;
}
