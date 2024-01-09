#include <ros/ros.h>
#include <people_msgs/People.h>

class PeopleDetectionTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/people_detection
     * 
     */

    public:
        PeopleDetectionTest()
        {
            // Create a subscriber object to get hand touch data
            sub = n.subscribe("/naoqi_driver/people", 1, &PeopleDetectionTest::peopleDetectionCallBack, this);
        }

        void peopleDetectionCallBack(const people_msgs::People& msg)
        {
            ROS_INFO_STREAM(" ------------ Printing people detection sensor data -----------\n\n");

            ROS_INFO_STREAM("Number of people detected: " << msg.people.size() << "\n" );

            // print the position of each person detected
            for (int i = 0; i < msg.people.size(); i++)
            {
                ROS_INFO_STREAM("Person " << i+1 << " position: " << msg.people[i].position << "\n" );
            }

            ROS_INFO_STREAM("---------------------------------------------------------\n\n");
        }


    private:
    
            // create a node
            ros::NodeHandle n;
    
            // create subscriber object
            ros::Subscriber sub;

};

int main(int argc, char** argv)
{
    // Initialize the ROS system 
    ros::init(argc, argv, "people_detection");

    // instanciate an object from the class HeadTouchTest
    PeopleDetectionTest peopleDetection;

    // Create a ROS loop rate object to set the frequency of the loop
    ros::Rate loop_rate(50);

    // While loop to keep the program running until Ctrl+C is pressed
    while (ros::ok())
    {
        // spin() function calls the callback function queue
        ros::spinOnce();

        // Sleep for the remaining time until we hit our 1 Hz rate
        loop_rate.sleep();
    }

    return 0;
}
