/*
 * File: Movement.cpp
 * Project: Project Swarm PX4
 * File Created: Tuesday, 19th February 2019 1:44:45 PM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief: This Class Is Responsible For All The Drone Movement.
 * -----
 * Last Modified: Tuesday, 19th February 2019 1:44:46 PM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

#include "Movement.h"

/**
 * @brief This Gets And Sets The Current State Of The Drone.
 *        That Is If It Is Connected Or Not.
 * 
 * @param &msg mavros_msgs::State // The Current State Message.
 * 
 * @return void
 */
void Movement::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    this->currentState = *msg;
}

/**
 * @brief This Gets And Sets The Longitude, Latitude and Altitude Variables.
 * 
 * @param msg sensor_msgs::NavSatFix // The Global Position Data Of The Drone.
 * 
 * @return void
 */
void Movement::locationCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    this->longitude = msg->longitude;
    this->latitude = msg->latitude;
    this->altitude = msg->altitude;
}

/**
 * @brief This Prints A Message To The Console.
 *
 * @param type string //The Type Of Messaged It Is. Either Info Or Error.
 * @param message string //The Messaged To Be Printed.
 *
 * @return void
 */
void Movement::display(string type, string message)
{
    std_msgs::String msg;
    std::stringstream tmp;
    tmp << droneID + " " + message;
    msg.data = tmp.str();

    if (type == "info")
    {
        ROS_INFO("%s", msg.data.c_str());
    }
    else if (type == "error")
    {
        ROS_ERROR("%s", msg.data.c_str());
    }
}

/**
 * @brief This Checks If The Drone Connected.
 * 
 * @return bool
 */
bool Movement::isConnected()
{
    bool valid = false;
    ros::Rate rate(this->hz);
    ros::Subscriber stateSubscriber = this->nodeHandler_.subscribe<mavros_msgs::State>(this->droneID + "/mavros/state", 10, &Movement::stateCallback, this);

    double time_start = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < time_start + this->elapsetime)
    {
        if (this->currentState.connected)
        {
            valid = true;
            this->display("info", "Is Connected");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return valid;
}

/**
 * @brief This Arms The Drone.
 * 
 * @return bool
 */
bool Movement::armDrone()
{
    bool valid = false;
    this->armingClient = this->nodeHandler_.serviceClient<mavros_msgs::CommandBool>(this->droneID + "/mavros/cmd/arming");

    mavros_msgs::CommandBool armCmd;
    armCmd.request.value = true;

    // double time_start = ros::Time::now().toSec();
    // while (ros::Time::now().toSec() < time_start + this->elapsetime)
    // {
    if (this->armingClient.call(armCmd) && armCmd.response.success)
    {
        valid = true;
        this->display("info", "Armed");
        // break;
    }
    {
        this->display("error", "ARM - Failed to Call Arming Service");
        // break;
    }
    // }
    return valid;
}

/**
 * @brief This Sets The Flight Mode Of The Drone.
 * 
 * @return bool
 */
bool Movement::setMode()
{
    bool valid = false;
    this->setModeClient = this->nodeHandler_.serviceClient<mavros_msgs::SetMode>(this->droneID + "/mavros/set_mode");

    this->set_mode.request.base_mode = 0;
    this->set_mode.request.custom_mode = mode;

    // double requestTime = ros::Time::now().toSec();
    // double time_start = ros::Time::now().toSec();
    // while (ros::Time::now().toSec() < time_start + this->elapsetime)
    // {
    // if (currentState.mode != this->mode && (ros::Time::now().toSec() < requestTime + 5.0))
    // {
    if (this->setModeClient.call(this->set_mode) && this->set_mode.response.mode_sent)
    {
        valid = true;
        this->display("info", mode + " Mode Enabled");
        // break;
    }
    else
    {
        this->display("error", mode + " Mode Disabled");
        // break;
    }
    // }
    // }
    return valid;
}

/**
 * @brief The Default Constructor Of The Class.
 */
Movement::Movement() = default;

/**
 * @brief This set's up the Movement class to be used.
 *
 * @param ros::NodeHandle &nodeHandler //A reference to the ROS main node.
 * @param droneID string //The Drone Name.
 * @param mode string //The Flight Mode.
 */
Movement::Movement(const ros::NodeHandle &nodeHandler, string droneID, string mode) : nodeHandler_(nodeHandler)
{
    this->mode = mode;
    this->droneID = droneID;

    this->stateSubscriber = this->nodeHandler_.subscribe<mavros_msgs::State>(this->droneID + "/mavros/state", 10, &Movement::stateCallback, this);
    this->posePublisher = this->nodeHandler_.advertise<geometry_msgs::PoseStamped>(this->droneID + "/mavros/setpoint_position/local", 10);
    this->armingClient = this->nodeHandler_.serviceClient<mavros_msgs::CommandBool>(this->droneID + "/mavros/cmd/arming");
    this->setModeClient = this->nodeHandler_.serviceClient<mavros_msgs::SetMode>(this->droneID + "/mavros/set_mode");
    ros::Subscriber longitudeSubscriber = this->nodeHandler_.subscribe<sensor_msgs::NavSatFix>(this->droneID + "/mavros/global_position/global", 10, &Movement::locationCallback, this);
}

/**
 * @brief The Destructor Of The Class.
 */
Movement::~Movement() = default;

/**
 * @brief This set's up the Movement class to be used.
 *
 * @param droneID string //The Drone Name.
 * @param mode string //The Flight Mode.
 *
 * @return void
 */
void Movement::Setup(const ros::NodeHandle &nodeHandler, string droneID, string mode)
{
    this->nodeHandler_ = nodeHandler;
    this->mode = mode;
    this->droneID = droneID;

    this->stateSubscriber = this->nodeHandler_.subscribe<mavros_msgs::State>(this->droneID + "/mavros/state", 10, &Movement::stateCallback, this);
    this->posePublisher = this->nodeHandler_.advertise<geometry_msgs::PoseStamped>(this->droneID + "/mavros/setpoint_position/local", 10);
    this->armingClient = this->nodeHandler_.serviceClient<mavros_msgs::CommandBool>(this->droneID + "/mavros/cmd/arming");
    this->setModeClient = this->nodeHandler_.serviceClient<mavros_msgs::SetMode>(this->droneID + "/mavros/set_mode");
    ros::Subscriber longitudeSubscriber = this->nodeHandler_.subscribe<sensor_msgs::NavSatFix>(this->droneID + "/mavros/global_position/global", 10, &Movement::locationCallback, this);
}
/**
 * @brief This Makes The Drone Take Off. Publishes To A ROS Topic Responsible
 *        For The Drone's Takeoff.
 *
 * @return bool
 */
bool Movement::takeoff()
{
    bool valid = false;
    if (this->status == true)
    {
        ros::ServiceClient takeoffClient = this->nodeHandler_.serviceClient<mavros_msgs::CommandTOL>(this->droneID + "/mavros/cmd/takeoff");

        mavros_msgs::CommandTOL takeoffCmd;
        takeoffCmd.request.longitude = this->longitude;
        takeoffCmd.request.latitude = this->latitude;
        takeoffCmd.request.altitude = 10;

        if (takeoffClient.call(takeoffCmd))
        {
            valid = true;
            this->display("info", "Takeoff Successful");
        }
        else
        {
            this->display("error", "Takeoff Unsuccessful");
        }
    }
    else
    {
        this->display("error", "Takeoff - Status Check Failed");
    }
    return valid;
}

/**
 * @brief This Makes The Drone Land. Publishes To A ROS Topic Responsible
 *        For The Drone's Landing.
 *
 * @return bool
 */
bool Movement::land()
{
    bool valid = false;
    if (this->status)
    {
        ros::ServiceClient landClient = this->nodeHandler_.serviceClient<mavros_msgs::CommandTOL>(this->droneID + "/mavros/cmd/land");

        mavros_msgs::CommandTOL landCmd;
        landCmd.request.longitude = this->longitude;
        landCmd.request.latitude = this->latitude;
        landCmd.request.altitude = this->altitude;

        if (landClient.call(landCmd))
        {
            valid = true;
            this->display("info", "Land Successful");
        }
        else
        {
            this->display("error", "Land Unsuccessful");
        }
    }
    else
    {
        this->display("error", "LAND - Status Check Failed");
    }
    return valid;
}

/**
 * @brief This Makes The Drone Hover At A set Altitude. Publishes To A ROS Topic
 *        Resposible For The Drone's Hovering.
 *
 *  @return bool
 */
bool Movement::hover(bool lock)
{
    bool valid = false;

    this->display("info", "Hovering");

    ros::Rate rate(this->hz);

    // wait for FCU connection
    while (ros::ok() && !this->currentState.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    this->pose.pose.position.x = 0;
    this->pose.pose.position.y = 0;
    this->pose.pose.position.z = 2;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        this->posePublisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    this->set_mode.request.custom_mode = this->mode;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (this->currentState.mode != this->mode &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (this->setModeClient.call(this->set_mode) &&
                this->set_mode.response.mode_sent)
            {
                this->display("info", "Offboard Mode Enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!this->currentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (this->armingClient.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    this->display("info", "Drone Armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (lock != true)
        {
            int c = this->GetKey();
            if (c != EOF)
            {
                switch (c)
                {
                case 65: // key up
                    this->pose.pose.position.z += 0.25;
                    this->display("info", "UP");
                    break;
                case 66: // key down
                    this->pose.pose.position.z += -0.25;
                    this->display("info", "DOWN");
                    break;
                case 67: // key right
                    this->pose.pose.position.y -= 0.25;
                    this->display("info", "RIGHT");
                    break;
                case 68: // key left
                    this->pose.pose.position.y += 0.25;
                    this->display("info", "LEFT");
                    break;
                }
            }
        }

        this->posePublisher.publish(this->pose);
        valid = true;
        ros::spinOnce();
        rate.sleep();
    }
    return valid;
}

void Movement::piloting(double pitch, double roll, double throttle)
{
    this->pose.pose.position.x += pitch;
    this->pose.pose.position.y += roll;
    this->pose.pose.position.z += throttle;
}

char Movement::GetKey()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
    {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
    {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0)
    {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
    {
        perror("tcsetattr ~ICANON");
    }
    return buf;
}