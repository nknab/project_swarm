/*
 * File: Tracking.h
 * Project: Project Swarm PX4
 * File Created: Thursday, 28th February 2019 4:20:45 PM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief: This The Header File For The Class Responsible For Tracking An
 *        Object Based On Color.
 * -----
 * Last Modified: Thursday, 28th February 2019 4:23:41 PM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

#ifndef PROJECT_TRACKING_H
#define PROJECT_TRACKING_H

//The Necessary Imports For the Class.
#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <thread>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include "../Movement/Movement.h"
#include "../FeedbackControl/FeedbackControl.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

//Defining The Scope For OpenCV And Standard Template Library.
using namespace cv;
using namespace std;

class Tracking
{

 private:
   //Declaring Of ROS Node Handler For The Class.
   ros::NodeHandle nodeHandler_;

   //Declaring And Instantiating The Classes and ROS Nodes Need For The Video Input From The Drone.
   image_transport::ImageTransport imageTransport_;
   //Declaring And Instantiating The Image Transport Subscriber.
   image_transport::Subscriber imageSubscriber_;
   //Declaring And Instantiating The Image Transport Publisher.
   image_transport::Publisher imagePublisher_;

   string droneID, mode = "";

   double kp = 0.0002;
   double ki = 0.000001;
   double kd = 0.004;

   double rollPrevError, throttlePrevError, offset, rollIntegral, throttleIntegral, targetValue = 0.0;

   bool movementState = false;
   /**
     * @brief This Removes Noise From The Video Being Streamed By Eroding And Dilating
     *        The Video Stream.
     *nodeHandler_(nodeHandler)
     * @param thresh Mat //The Multi Dimensional Array Of The Image Being Tracked.
     *
     * @return void
     */
   void removeNoise(cv::Mat &thresh);

   /**
     * @brief This Converts An Integer To A String.
     *
     * @param number int //Number Being Converted.
     *
     * @return string The Converted Number.
     */
   string toString(int number);

   /**
     * @brief This Detects The Object And Then Tracks The Object Based On Color.
     *
     * @param x int //The X Coordinate (Center Point) Of The Object Being Tracked.
     * @param y int //The Y Coordinate (Center Point) Of The Object Being Tracked.
     * @param threshold cv::Mat //The Multi Dimensional Array Of The Image Being Tracked.
     * @param cameraFeed cv::Mat //The Multi Dimensional Array Of The Video Stream.
     *
     * @return void
     */
   void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);

   /**
     * @brief This Coordinates The Entire Class And Makes The Drone To Be Able To Track
     *        The Object.
     *
     * @param msg sensor_msgs::ImageConstPtr //This Is The Message Being Published To
     *            The Image Tranport Topic.
     *
     * @return void
     */
   void trackingCallBack(const sensor_msgs::ImageConstPtr &msg);

 protected:
   //Declaring And Instantiating A Double To Hold The Throttle and Roll Values After Computation.
   double throttle, roll = 0;

   //Declaring A String Variable To Hold The Name Of The Video Stream Window.
   string window = " - Drone Video Stream";

   //Declaring A 1x6 Array To Hold The Colour To Track.
   //[H-Min, H-Max, S-Min, S-Max, V-Min, V-Max]
   int hsv[6];

   //Declaring And Instantiating A 1x3 Array To Hold The Various Object Properties To Be Tracked.
   //[Number Of Objects, Minimum Object Area, Maximum Object Area]
   //Maximum Object Area = (Frame Height * Frame Height) / 1.5
   int objectProperties[3] = {50, 10 * 10, int(640 * 368 / 1.5)};

   //Instance Of The Movement Class.
   Movement movement;

   FeedbackControl feedbackControl;

 public:
   /**
     * @brief The Constructor Of The Class. It Has A Base Initializer That Automatically
     *        Starts The Object Tracking.
     *
     * @param droneID string //The Drone Name(Topic Root).
     * @param hsv int[] //The HSV Color Boundaries.
     */
   explicit Tracking(const ros::NodeHandle &nodeHandler, string droneID, string mode, int hsv[]);

   /**
     * @brief The Destructor Of The Class. This Basically Stops The Object Tracking,
     *        Lands The Drone And Closes The Video Streaming Window.
     */
   virtual ~Tracking();
};

#endif //PROJECT_TRACKING_H