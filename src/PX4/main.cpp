/*
 * File: main.cpp
 * Project: Project Swarm PX4
 * File Created: Tuesday, 19th February 2019 1:23:22 PM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief:
 * -----
 * Last Modified: Tuesday, 19th February 2019 1:23:23 PM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

//The Necessary Imports For the Class.
#include <thread>
#include <ros/ros.h>
#include "Movement/Movement.h"
#include "Tracking/Tracking.h"

using namespace std;

void trackingthread(ros::NodeHandle nodeHandler, string droneID, string mode, int colour[6])
{
    Tracking track(nodeHandler, droneID, mode, colour);
    ros::spin();
}

int main(int argc, char **argv)
{
    string droneID1 = "ps1";
    string droneID2 = "ps2";
    string droneID3 = "ps3";
    string mode = "OFFBOARD";

    //HSV For Red
    int red[6] = {0, 10, 70, 255, 50, 255};

    // HSV For Yellow
    int yellow[6] = {20, 30, 100, 255, 100, 255};

    //HSV For Green
    int green[6] = {29, 64, 86, 255, 6, 255};

    // HSV For Blue
    int blue[6] = {110, 130, 50, 255, 50, 255};

    ros::init(argc, argv, "Project_Swrm");
    ros::NodeHandle nodeHandler("");

    // Tracking ps1_track(nodeHandler, droneID1, mode, red);

    ros::Rate r(10);

    Movement ps1_move(nodeHandler, droneID1, mode);
    // Movement ps2_move(nodeHandler, droneID2, mode);

    std::thread t1{[&]() {
        ps1_move.hover(false);
        ros::spin();
    }},
        t2{[&]() {
            Tracking ps2_track(nodeHandler, droneID2, mode, red);
            ros::spin();
        }};
    // ,
    // t3{[&]() {
    //     Tracking ps3_track(nodeHandler, droneID3, mode, yellow);
    //     ros::spin();
    // }};

    t1.join();
    t2.join();
    // t3.join();

    return 0;
}
