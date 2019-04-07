/**
 * @author: nknab
 * @date: Thursday, 6th December 2018.
 * @time: 4:42 PM
 * @project: Project Swarm
 * @version: 1.0
 * @brief: This Class Is Responsible For All The Drone Movement.
 */

//Import Of Class Header File.
#include "Movement.h"


/**
 * @brief This Prints A Message To The Console.
 *
 * @param message string //The Messaged To Be Printed.
 *
 * @return void
 */
void Movement::display(string message) {
    std_msgs::String msg;
    std::stringstream tmp;
    tmp << droneID + message;
    msg.data = tmp.str();

    ROS_INFO("%s", msg.data.c_str());
}

/**
 * @brief The Constructor Of The Class.
 */
Movement::Movement() = default;

/**
 * @brief The Destructor Of The Class.
 */
Movement::~Movement() = default;

/**
 * @brief This Assigns The Drone Name To The Member Variable droneID.
 *
 * @param droneID string //The Drone Name.
 *
 * @return void
 */
void Movement::setup(string droneID) {
    this -> droneID = droneID;
}

/**
 * @brief This Makes The Drone Take Off. Publishes To A Ros Topic Responsible
 *        For The Drone's Takeoff.
 *
 * @return void
 */
void Movement::takeoff() {
    publisher = nodeHandler.advertise<std_msgs::Empty>("/" + droneID + "/takeoff", 1);
    std_msgs::Empty empty_msg;

    double time_start = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < time_start + duration) {
        publisher.publish(empty_msg);
        ros::spinOnce();
    }
    display(" launched successfully");
}

/**
 * @brief This Makes The Drone Land. Publishes To A Ros Topic Responsible
 *        For The Drone's Landing.
 *
 * @return void
 */
void Movement::land() {
    publisher = nodeHandler.advertise<std_msgs::Empty>("/" + droneID + "/land", 1);
    std_msgs::Empty empty_msg;

    double time_start = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < time_start + duration) {
        publisher.publish(empty_msg);
        ros::spinOnce();
    }
    display(" landed successfully");
}

/**
 * @brief This Makes The Drone Land In Case Of Emergency. Publishes To A Ros
 *        Topic Responsible For The Drone's Emergency Landing.
 *
 * @return void
 */
void Movement::emergency() {
    publisher = nodeHandler.advertise<std_msgs::Empty>("/" + droneID + "/reset", 1);
    std_msgs::Empty empty_msg;

    double time_start = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < time_start + duration) {
        publisher.publish(empty_msg);
        ros::spinOnce();
    }
    display(" emergency landing");
}

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
void Movement::piloting(double pitch, double roll, double throttle, double yaw) {
    geometry_msgs::Twist coordinates;

    publisher = nodeHandler.advertise<geometry_msgs::Twist>("/" + droneID + "/cmd_vel", 1);

    coordinates.linear.x = pitch;
    coordinates.linear.y = roll;
    coordinates.linear.z = throttle;
    coordinates.angular.z = yaw;

    double time_start = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < time_start + duration) {
        publisher.publish(coordinates);
        ros::spinOnce();
    }
    std_msgs::String msg;
    std::stringstream tmp;
    tmp << "Pitch: " << pitch << " Roll: " << roll << " Throttle: " << throttle << " Yaw: " << yaw << " have been sent to " << droneID;
    msg.data = tmp.str();

    ROS_INFO("%s", msg.data.c_str());
}