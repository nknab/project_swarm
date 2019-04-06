#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "TrackObject/TrackObject.h"
#include "Movement/Movement.h"

int main(int argc, char **argv) {

    string window = "Drone Video Stream";
    int colour[6] = {29, 64, 86, 255, 6, 255};
//    int colour[6] = {21, 30, 200, 255, 102, 255};


    string droneID = "ps1";
    string ps1IP = "10.188.240.147";

    char *cmd = const_cast<char *>("gnome-terminal --geometry=42x9+1020+20 -x sh -c \"cd ~ && echo a | sudo -S sudo systemctl start firmwared.service && sudo firmwared; bash\"");
    system(cmd);
    sleep(5);

    string tmp = "gnome-terminal --geometry=42x9+1020+222 -x sh -c \"cd ~ && sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/empty.world /opt/parrot-sphinx/usr/share/sphinx/drones/ps1.drone::stolen_interface=wlan0:eth0:" + ps1IP + "; bash\"";
    cmd = new char[tmp.length() + 1];
    std::strcpy(cmd,tmp.c_str());
    system(cmd);
    sleep(25);

    cmd = const_cast<char *>("gnome-terminal --geometry=42x9+1020+420 -x sh -c \"cd ~ && roslaunch bebop_driver ps1.launch; bash\"");
    system(cmd);
    sleep(25);


    ros::init(argc, argv, "Project_Swarm");

    Movement ps1_move;
    ps1_move.setup(droneID);

//    TrackObject ps1_to(droneID, window, colour);
//    ros::spin();

//    /*
    ps1_move.takeoff();
    sleep(15);
    for (int x = 0; x < 2; x++) {
        ps1_move.piloting(0.0, 0.0, 0.0, 1.0);
        sleep(5);
    }
    ps1_move.land();
//    */
//    ps1_move.land();

//    ps1_move.emergency();

    return 0;
}