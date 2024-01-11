# include "pepper_interface_tests/actuatorTest.h"

Actuator::Actuator(std::string& actuatorName) : actuatorName(actuatorName) {
    std::string topicName = configuration::extractTopic(actuatorName);

    try {
        client = boost::make_shared<ControlClient>(topicName, true);
        client->waitForServer();
    }
    catch (std::runtime_error& e) {
        ROS_FATAL_STREAM(e.what());
        Utility::promptAndExit(1);
    }
}

void Actuator::setTrajectory(const std::vector<std::string>& jointNames, double duration, 
                         std::vector<double> positions){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
}

void Actuator::testJoint(ControlClientPtr& client, const std::vector<std::string>& jointNames, std::vector<std::vector<double>> duration, 
                        std::vector <double> maxPosition, std::vector<double> minPosition, 
                        std::vector <double> homePosition ,std::vector<std::vector<double>> velocity){
    
    std::vector<double> position(maxPosition.size(), 0.0);

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        setTrajectory(jointNames, duration[i][0], position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        setTrajectory(jointNames, duration[i][1], position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        setTrajectory(jointNames, duration[i][2], position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("Moving to the Home position");
    double homeDuration = 2.0;
    setTrajectory(jointNames, homeDuration, homePosition);
}

std::vector<std::vector<double>> Actuator::calculateDuration (std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity){
    
    // Initialize the duration vector similar to the velocity vector
    std::vector<std::vector<double>> duration(velocity.size(), std::vector<double>(velocity[0].size(), 0.0));
    
    // Calculate the duration for each joint check if the velocity is 0 or not
    for (int i = 0; i < homePosition.size(); ++i){
        // Calculate the duration for the first part of the trajectory
        duration[i][0] = std::fabs(minPosition[i] - homePosition[i]) / velocity[i][0];
        // Calculate the duration for the second part of the trajectory
        duration[i][1] = std::fabs(maxPosition[i] - minPosition[i]) / velocity[i][1];
        // Calculate the duration for the third part of the trajectory
        duration[i][2] = std::fabs(homePosition[i] - maxPosition[i]) / velocity[i][2]; 
        std::cout<<duration[i][2]<<std::endl; 
    }

    return duration;
}

HeadActuator::HeadActuator(std::string actuatorName) : Actuator(actuatorName) {
    jointNames = {"HeadPitch", "HeadYaw"};
    maxPosition = {0.4451, 2.0857};
    minPosition = {-0.7068, -2.0857};
    homePosition = {-0.2, 0.012271};
    velocity = {{1.5, 1.5, 1.5},{1.2, 1.2, 1.2}};
    duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);
}

ArmActuator::ArmActuator(std::string actuatorName) : Actuator(actuatorName) {
    if (actuatorName == "rarm"){
        jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
        maxPosition = {2.0857,  -0.0087,  1.5620,  2.0857,  1.8239};
        minPosition = {-2.0857, -1.5620 , 0.0087, -2.0857, -1.5620};
        homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
        velocity = {{1.5, 1.5, 0.1}, {1.2, 0.8, 0.15},{0.1, 0.8, 1.2}, {2.0, 1.5, 0.2}, {1.8, 1.8, 1.8}};
    }
    else if (actuatorName == "larm"){
        jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
        maxPosition = {2.0857,  0.0087,  -1.5620, -2.0857,  -1.8239};
        minPosition = {-2.0857, 1.5620 , -0.0087,  2.0857,   1.8239};
        homePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};
        velocity = {{1.5, 1.5, 0.1},{1.2, 0.8, 0.15},{0.1, 0.9, 1.2},{2.1, 1.5, 0.2},{1.8, 1.8, 1.9}};
    }
    duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);

}

HandActuator::HandActuator(std::string actuatorName) : Actuator(actuatorName) {
    if (actuatorName == "rhand"){
        jointNames = {"RHand"};
        maxPosition = {1.0};
        minPosition = {0.0};
        homePosition = {0.66608};
        velocity = {{1.0, 1.0, 1.0}};
    }
    else if (actuatorName == "lhand"){
        jointNames = {"LHand"};
        maxPosition = {1.0};
        minPosition = {0.0};
        homePosition = {0.66608};
        velocity = {{1.0, 1.0, 1.0}};
    }
    duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);
}

LegActuator::LegActuator(std::string actuatorName) : Actuator(actuatorName) {
    jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    maxPosition = {1.0385,   0.5149,   0.5149};
    minPosition = {-1.0385, -0.5149 , -0.5149};
    homePosition = {-0.0107, -0.00766, 0.03221};
    velocity = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
    duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);
}

WheelActuator::WheelActuator(ros::NodeHandle& nh, std::string actuatorName) {
    std::string topicName = configuration::extractTopic("wheel");
    pub = nh.advertise<geometry_msgs::Twist>(topicName, 1000);
}

void WheelActuator::publishVelocity(ros::Publisher& pub, geometry_msgs::Twist& msg, ros::Rate& rate, double duration){
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(duration); 
    ros::Time endTime = startTime + waitTime;
    // Publish the trajectory for 1 seconds
    while(ros::ok() && ros::Time::now() < endTime) {
        pub.publish(msg);
        rate.sleep();
    }
}

void WheelActuator::testWheel(){
    ros::Rate rate(50);
    
     ROS_INFO_STREAM("-------[START WHEEL CONTROL TEST]--------");
   /* [1] THIS SECTION PUBLISHES A LINEAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[LINEAR VELOCITY START] Publishing linear velocity on the cmd vel started.");
   
   // Initialize the message with 0 linear velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity
   publishVelocity(pub, msg, rate, 1);

   // Publish a fixed positive linear velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.linear.x = 0.5;

   // Publish the positive velocity 
   publishVelocity(pub, msg, rate, 2);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);

   // Publish a fixed negative linear velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.linear.x = -0.5;

   // Publish the negative velocity 
   publishVelocity(pub, msg, rate, 2);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);
   
   ROS_INFO_STREAM("[LINEAR VELOCITY END] Publishing linear velocity ended.");
   
   /* [2] THIS SECTION PUBLISHES AN ANGULAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[ANGULAR VELOCITY START] Publishing angular velocity on the cmd vel started.");
   
   // Initialize the message with 0 angular velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);

   // Publish a fixed positive angular velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.angular.z = 1.57;

   // Publish the positive velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);

   // Publish a fixed negative angular velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.angular.z = -1.57;

   // Publish the negative velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);
   
   ROS_INFO_STREAM("[ANGULAR VELOCITY END] Publishing angular velocity ended.");
    
   // Print success message
   ROS_INFO_STREAM("[SUCCESS] Wheel control test completed.");
}

std::string configuration::extractTopic(std::string key){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                  // configuration path
    std::string configPathFile;                         // configuration path and filename
    
    std::string platformKey = "platform";                             // platform key 
    std::string robotTopicKey = "robottopics";                        // robot topic key
    std::string simulatorTopicKey = "simulatortopics";                // simulator topic key

    std::string platformValue;                                        // platform value
    std::string robotTopicValue;                                      // robot topic value
    std::string simulatorTopicValue;                                  // simulator topic value
    std::string mode;                                                 // mode value
    
    std::string topicFileName;                                        // topic filename
    std::string topicPath;                                            // topic filename path
    std::string topicPathFile;                                        // topic with path and file 

    std::string topic_value = "";                                     // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        configPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        configPath = "..";
    #endif

    // set configuration path
    configPath += "/config/";
    configPathFile = configPath;
    configPathFile += configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        Utility::promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == platformKey){ platformValue = paramValue;}
        
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}

        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}

    }

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topicFileName = "simulatorTopics.dat"; }
    else if (platformValue == "robot") { topicFileName = "pepperTopics.dat"; }

    // Construct the full path of the topic file
    #ifdef ROS
        topicPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();

    #else
        topicPath = "..";

    #endif

    // set topic path
    topicPath += "/data/";
    topicPathFile = topicPath;
    topicPathFile += topicFileName;

    if (debug) printf("Topic file is %s\n", topicPathFile.c_str());

    // Open topic file
    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()){
        printf("Unable to open the topic file %s\n", topicPathFile.c_str());
        Utility::promptAndExit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topicFile, topicLineRead)){
        std::istringstream iss(topicLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        
        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);

        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        Utility::promptAndExit(1);
    }

    return topic_value;
}

std::string configuration::extractMode(){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                  // configuration path
    std::string configPathFile;                         // configuration path and filename
    
    std::string modeKey = "mode";                             // mode key 

    std::string modeValue;                                    // mode value
    
    // Construct the full path of the configuration file
    #ifdef ROS
        configPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        configPath = "..";
    #endif

    // set configuration path
    configPath += "/config/";
    configPathFile = configPath;
    configPathFile += configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        Utility::promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == modeKey){ modeValue = paramValue;}
    }
    configFile.close();

    // verify the modeValue is not empty
    if (modeValue == ""){
        printf("Unable to find a valid mode.\n");
        Utility::promptAndExit(1);
    }
    return modeValue;
}

std::vector<std::string> configuration::extractTests(std::string set){
    bool debug = false;   // used to turn debug message on
    
    std::string inputFileName;                                  // input filename
    std::string inputPath;                                  // input path
    std::string inputPathFile;                         // input path and filename
    
    std::vector<std::string> testName;
    std::string flag;

    if (set == "actuator"){
        inputFileName = "actuatorTestInput.ini";
    }
    else if (set == "sensor"){
        inputFileName = "sensorTestInput.ini";
    }
    else {
        printf("unable to identify the test.\n");
    }

    // Construct the full path of the input file
    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        std::cout<<inputPath<<std::endl;
    #else
        inputPath = "..";
    #endif
    
    inputPath += "/config/";
    inputPathFile = inputPath;
    inputPathFile += inputFileName;

    if (debug) printf("Input file is %s\n", inputPathFile.c_str());

    // Open input file
    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()){
        printf("Unable to open the input file %s\n", inputPathFile.c_str());
        Utility::promptAndExit(1);
    }

    std::string inpLineRead;  // variable to read the line in the file
    
    std::string paramKey, paramValue;
    // Get key-value pairs from the input file
    while(std::getline(inputFile, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue); // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower); // convert to lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower); // convert to lower case

        if (paramValue == "true"){ testName.push_back(paramKey);}
    }
    inputFile.close();

    return testName;
}

void Utility::promptAndExit(int status){
    printf("Press any key to exit  ... \n");
    getchar();
    exit(status);
}

void Utility::promptAndContinue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

void ActuatorTestApplication::runTest() {
    
    std::vector<std::string> testNames = configuration::extractTests("actuator");
    std::string mode = configuration::extractMode();

    std::cout << "Mode: " << mode << std::endl;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
    }

    std::vector<std::thread> threads;
    for (const auto& name : testNames) {
        if (mode == "parallel") {
            std::vector<std::thread> threads;
            for (const auto& name : testNames) {
                if (name == "head") {
                    threads.emplace_back(&ActuatorTestApplication::testHead, this, name);
                }
                else if (name == "rarm") {
                    threads.emplace_back(&ActuatorTestApplication::testArm, this, name);
                }
                else if (name == "larm") {
                    threads.emplace_back(&ActuatorTestApplication::testArm, this, name);
                }
                else if (name == "rhand") {
                    threads.emplace_back(&ActuatorTestApplication::testHand, this, name);
                }
                else if (name == "lhand") {
                    threads.emplace_back(&ActuatorTestApplication::testHand, this, name);
                }
                else if (name == "leg") {
                    threads.emplace_back(&ActuatorTestApplication::testLeg, this, name);
                }
                else if (name == "wheel") {
                    threads.emplace_back(&ActuatorTestApplication::testWheel, this, name);
                }
                else {
                    std::cout << "Invalid test name: " << name << std::endl;
                }
        }
        for (auto& thread : threads) {
            thread.join();
        }

        }
        else if (mode == "sequential") {
            if (name == "head") {
                testHead(name);
            }
            else if (name == "rarm") {
                testArm(name);
            }
            else if (name == "larm") {
                testArm(name);
            }
            else if (name == "rhand") {
                testHand(name);
            }
            else if (name == "lhand") {
                testHand(name);
            }
            else if (name == "leg") {
                testLeg(name);
            }
            else if (name == "wheel") {
                testWheel(name);
            }
            else {
                std::cout << "Invalid test name: " << name << std::endl;
            }
        }
        else {
            std::cout << "Invalid mode: " << mode << std::endl;
        }
}
}
