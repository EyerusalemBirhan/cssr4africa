# include "pepper_interface_tests/sensorTest.h"

Sensors(ros::NodeHandle& nh, const std::string& sensorName) 
: nh(nh), topicName(sensorName), output(true), timeDuration(10) {}

RgbCamera::RgbCamera(ros::NodeHandle& nh, const std::string& sensorName): Sensors(nh, sensorName) {
    string topicName = coniguration::extractTopic(sensorName);

    ROS_INFO_STREAM("Subscribing to topic " << topicName << "\n");
    ros::Duration(1).sleep();

    // create an image_transport object
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, &rgbCamera::rgbCameraMessageReceived, this);

    // Listen for incoming messages and execute callback function
    ros::Rate rate(30);
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(timeDuration);

    while(ros::Time::now() - startTime < waitTime) {
        ros::spinOnce();
        rate.sleep();
    }

    cv.destroyWindow(sensorName);
}

void RgbCamera::rgbCameraMessageReceived(sensor_msgs::ImageConstPtr& msg, std::string& sensorName) {
    // Extract images attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    ROS_INFO_STREAM("Received image of size " << imgWidth << "x" << imgHeight << "\n");

    // Write the message received in an output file if the output varialbe is trure
    if (output == true){
        string path;

        #ifdef ROS 
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "../";
        #endif

        // compose the output file name
        path += "/data/sensroTestOutput.dat";

        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            Utility::promptAndExit(1);
        }

        // write on the output file
        out_of << "[MESSAGES] ---- " << sensorName << " ----\n\n";
        out_of << "[MESSAGES] Printing" << sensorName << "information received.\n";
        out_of << "Image Width: " << imgWidth << "\n";
        out_of << "Image Height: " << imgHeight << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

std::string Configuration::extractTopic(std::string key){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                  // configuration path
    std::string configPathFile;                         // configuration path and filename
    
    std::string platformKey = "platform";                     // platform key 
    std::string robotTopicKey = "robotTopics";                // robot topic key
    std::string simulatorTopicKey = "simulatorTopics";        // simulator topic key

    std::string platformValue;                                // platform value
    std::string robotTopicValue;                              // robot topic value
    std::string simulatorTopicValue;                          // simulator topic value
    
    std::string topicFileName;                                   // topic filename
    std::string topicPath;                                   // topic filename path
    std::string topicPathFile;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

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
        
        if (paramKey == platformKey){ platformValue = paramValue;}
        
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}

        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}
    }
    configFile.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topicFileName.c_str());

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
        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }
    topicFile.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        Utility::promptAndExit(1);
    }
    return topic_value;
}

std::string Configuration::extractMode(){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                        // configuration path
    std::string configPathFile;                                    // configuration path and filename
    
    std::string modeKey = "mode";                                  // mode key 
    std::string modeValue;                                         // mode value

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

std::vector<std::string> Configuration::extractTests(std::string set){
    bool debug = false;   // used to turn debug message on
    
    std::string inputFileName;                                  // input filename
    std::string inputPath;                                  // input path
    std::string inputPathFile;                         // input path and filename
    
    std::vector<std::string> testName;
    std::string flag;

    if (set == "actuator"){
        inputFileName = "actuatorTestInput.ini";
    }
    else{
        inputFileName = "sensorTestInput.ini";
    }

    // Construct the full path of the input file
    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        inputPath = "..";
    #endif

    // set input path
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
    
    std::string paramKey, paramValue; // variables to keep the key value pairs read

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
    std::cout << "Press any key to exit...\n";
    std::cin.get();
    exit(status);
}

void Utility::promptAndContinue(){
    std::cout << "Press any key to continue...\n";
    std::cin.get();
}

void SensorTestApplication::runTests(){
    std::vector<std::string> testNames = Configuration::extractTests("sensor");
    std::string mode = Configuration::extractMode();

    std::cout<<"Mode: "<<mode<<"\n";

    std::time_t start_t = std::time(0);
    std::tm* start_now = std::localtime(&start_t);
    char start_buf[50];

    strftime(start_buf, sizeof(start_buf), "%Y-%m-%d.%X", start_now);

    std::string path;
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
        Utility::promptAndExit(1);
    }

    out_of << "[TESTING] ############ SENSORS ############\n\n";
    out_of << "[START TIME] " << start_buf << "\n";

    out_of.close();

    if (mode == "sequential") {
        for (auto test:testNames){
            if (test == "frontCamera"){
                testrgbCamera(test);
            }
            else {
                std::cout << "Test " << test << " not implemented yet.\n";
            }
            }
        }
    char end_buf[50];
    
    std::time_t end_t = std::time(0);
    std::tm* end_now = std::localtime(&end_t);
    strftime(end_buf, sizeof(end_buf), "%Y-%m-%d.%X", end_now);

    out_of.open(path.c_str(), ofstream::app);    
    out_of << "[END TIME] " << end_buf << "\n\n";
    out_of.close();

}

