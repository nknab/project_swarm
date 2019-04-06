/*
 * File: Movement.h
 * Project: Project Swarm PX4
 * File Created: Tuesday, 19th February 2019 1:45:33 PM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief: This The Header File For The Class Responsible For All The Drone Movement.
 * -----
 * Last Modified: Saturday, 23rd February 2019 11:45:19 AM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

#ifndef PROJECT_MOVEMENT_H
#define PROJECT_MOVEMENT_H

//The Necessary Imports For the Class.
#include <string>
#include <fcntl.h>
#include <ros/ros.h>
#include <termios.h>
#include "std_msgs/String.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>

//Defining The Scope For Standard Template Library.
using namespace std;

class Movement
{

private:
  //Declaring Of ROS Node Handler and Publisher For The Class.
  ros::NodeHandle nodeHandler_;
  ros::ServiceClient setModeClient;
  ros::ServiceClient armingClient;
  mavros_msgs::SetMode set_mode;
  ros::Publisher posePublisher;
  ros::Subscriber stateSubscriber;
  //Declaring And Instantiating The Various Variables Of The Class..
  mavros_msgs::State currentState;

  string droneID, mode = "";

  double elapsetime = 30.0;
  double hz = 20.0;
  double longitude, latitude, altitude = 0.000000;

  bool status = false;

  geometry_msgs::PoseStamped pose;

  /**
     * @brief This Gets And Sets The Current State Of The Drone.
     *        That Is If It Is Connected Or Not.
     * 
     * @param &msg mavros_msgs::State // The Current State Message.
     * 
     * @return void
     */
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);

  /**
     * @brief This Gets And Sets The Longitude, Latitude and Altitude Variables.
     * 
     * @param msg sensor_msgs::NavSatFix // The Global Position Data Of The Drone.
     * 
     * @return void
     */
  void locationCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

  /**
     * @brief This Prints A Message To The Console.
     *
     * @param type string //The Type Of Messaged It Is. Either Info Or Error.
     * @param message string //The Messaged To Be Printed.
     *
     * @return void
     */
  // while (ros::ok())
  // {
  //     if (this->currentState.mode != this->mode && (ros::Time::now() - last_request > ros::Duration(5.0)))
  //     {
  //         if (this->setMode())
  //         {
  //             continue;
  //         }
  //         last_request = ros::Time::now();
  //     }
  //     else
  //     {
  //         if (!this->currentState.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
  //         {
  //             if (this->armDrone())
  //             {
  //                 continue;
  //             }
  //             last_request = ros::Time::now();
  //         }
  //     }

  //     posePublisher.publish(this->pose);

  //     ros::spinOnce();
  //     rate.sleep();
  // }
  void display(string type, string message);

  /**
     * @brief This Checks If The Drone Connected.
     * 
     * @return bool
     */
  bool isConnected();

  /**
     * @brief This Arms The Drone.
     * 
     * @return bool
     */
  bool armDrone();

  /**
     * @brief This Sets The Flight Mode Of The Drone.
     * 
     * @return bool
     */
  bool setMode();

public:
  /**
     * @brief The Default Constructor Of The Class.
     */
  explicit Movement();

  /**
     * @brief The Constructor Of The Class.
     *
     * @param ros::NodeHandle &nodeHandler //A reference to the ROS main node.
     * @param droneID string //The Drone Name.
     * @param mode string //The Flight Mode.
     */
  explicit Movement(const ros::NodeHandle &nodeHandler, string droneID, string mode);

  /**
     * @brief The Destructor Of The Class.
     */
  virtual ~Movement();

  /**
     * @brief This set's up the Movement class to be used.
     *
     * @param ros::NodeHandle &nodeHandler //A reference to the ROS main node.
     * @param droneID string //The Drone Name.
     * @param mode string //The Flight Mode.
     *
     * @return void
     */
  void Setup(const ros::NodeHandle &nodeHandler, string droneID, string mode);

  /**
     * @brief This Makes The Drone Take Off. Publishes To A ROS Topic Responsible
     *        For The Drone's Takeoff.
     *
     * @return bool
     */
  bool takeoff();

  /**
     * @brief This Makes The Drone Land. Publishes To A ROS Topic Responsible
     *        For The Drone's Landing.
     *
     * @return bool
     */
  bool land();

  /**
     * @brief This Makes The Drone Hover At A set Altitude. Publishes To A ROS Topic
     *        Resposible For The Drone's Hovering.
     *
     *  @return bool
     */
  bool hover(bool lock);

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
  void piloting(double pitch, double roll, double throttle);

  char GetKey();
};

#endif //PROJECT_MOVEMENT_H