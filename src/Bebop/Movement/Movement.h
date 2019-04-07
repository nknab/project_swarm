/**
* @author: nknab
* @date: Thursday, 6th December 2018.
* @time: 4:42 PM
* @project: Project Swarm
* @version: 1.0
* @brief: This The Header File For The Class Responsible For All The Drone
*         Movement.
*/

#ifndef PROJECT_MOVEMENT_H
#define PROJECT_MOVEMENT_H

//The Necessary Imports For the Class.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <sstream>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

//Defining The Scope For Standard Template Library.
using namespace std;

class Movement {

private:
    //Declaring Of ROS Node Handler and Publisher For The Class.
    ros::NodeHandle nodeHandler;
    ros::Publisher publisher;

    //Declaring Of A String Variable To Hold The Drone Name.
    string droneID;

    //Declaring And Instantiating A Variable For The Time Frame Of Publishing To A Topic.
    double duration = 5.0;

    /**
     * @brief This Prints A Message To The Console.
     *
     * @param message string //The Messaged To Be Printed.
     *
     * @return void
     */
    void display(string message);


public:
    /**
     * @brief The Constructor Of The Class.
     */
    explicit Movement();

    /**
     * @brief The Destructor Of The Class.
     */
    virtual ~Movement();

    /**
     * @brief This Assigns The Drone Name To The Member Variable droneID.
     *
     * @param droneID string //The Drone Name.
     *
     * @return void
     */
    void setup(string droneID);

    /**
     * @brief This Makes The Drone Take Off. Publishes To A Ros Topic Responsible
     *        For The Drone's Takeoff.
     *
     * @return void
     */
    void takeoff();

    /**
     * @brief This Makes The Drone Land. Publishes To A Ros Topic Responsible
     *        For The Drone's Landing.
     *
     * @return void
     */
    void land();

    /**
     * @brief This Makes The Drone Land In Case Of Emergency. Publishes To A Ros
     *        Topic Responsible For The Drone's Emergency Landing.
     *
     * @return void
     */
    void emergency();

    /**
     * @brief This Makes The Drone Fly In A Specific Direction. Publishes To A
     *        Ros Topic Responsible For The Drone's Piloting.
     *
     * @param pitch double //The Pitch Value (Forward or Backward).
     * @param roll double //The Roll Value (Left or Right).
     * @param throttle double //The Throttle Value (Upward or Downward).
     * @param yaw double //The Yaw Value (Rotate Left or Rotate Right).
     *
     * @return void
     */
    void piloting(double pitch, double roll, double throttle, double yaw);

};


#endif //PROJECT_MOVEMENT_H
