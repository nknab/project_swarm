/*
 * File: Tracking.cpp
 * Project: Project Swarm PX4
 * File Created: Thursday, 28th February 2019 4:22:12 PM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief: This Class Is Responsible For Tracking An Object Based On Color.
 * -----
 * Last Modified: Thursday, 28th February 2019 4:22:12 PM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

//Import Of Class Header File.
#include "Tracking.h"

/**
 * @brief This Removes Noise From The Video Being Streamed By Eroding And Dilating
 *        The Video Stream.
 *
 * @param thresh Mat //The Multi Dimensional Array Of The Image Being Tracked.
 *
 * @return void
 */
void Tracking::removeNoise(cv::Mat &thresh)
{
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));

    cv::erode(thresh, thresh, erodeElement);
    cv::erode(thresh, thresh, erodeElement);

    cv::dilate(thresh, thresh, dilateElement);
    cv::dilate(thresh, thresh, dilateElement);
}

/**
 * @brief This Converts An Integer To A String.
 *
 * @param number int //Number Being Converted.
 *
 * @return string The Converted Number.
 */
string Tracking::toString(int number)
{
    std::stringstream temp;
    temp << number;

    return temp.str();
}

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
void Tracking::trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{

    Mat temp;
    threshold.copyTo(temp);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double refArea = 0;
    bool objectFound = false;
    if (!hierarchy.empty())
    {
        int numberOfObjects = static_cast<int>(hierarchy.size());

        if (numberOfObjects <= objectProperties[0])
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                if (area > objectProperties[1] && area < objectProperties[2] && area > refArea)
                {
                    x = int(moment.m10 / area);
                    y = int(moment.m01 / area);
                    objectFound = true;
                    refArea = area;
                }
                else
                {
                    objectFound = false;
                }
            }
            if (objectFound)
            {
                cv::putText(cameraFeed, "Drone Detected!", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                cv::circle(cameraFeed, Point(x, y), 20, Scalar(0, 255, 0), 2);
                cv::putText(cameraFeed, toString(x) + "," + toString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
            }
            else
            {
                cv::putText(cameraFeed, "Drone Not Detected", Point(0, 50), 1, 2, Scalar(255, 0, 0), 2);
            }
        }
        else
        {
            cv::putText(cameraFeed, "Drone Not Detected", Point(0, 50), 1, 2, Scalar(255, 0, 0), 2);
        }
    }
}

/**
 * @brief This Coordinates The Entire Class And Makes The Drone To Be Able To Track
 *        The Object.
 *
 * @param msg sensor_msgs::ImageConstPtr //This Is The Message Being Published To
 *            The Image Tranport Topic.
 *
 * @return void
 */
void Tracking::trackingCallBack(const sensor_msgs::ImageConstPtr &msg)
{

    ros::Rate rate(20.0);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("CV_Bridge Exception: %s", e.what());
        return;
    }

    cv::Mat input = cv_ptr->image, hsv_image, threshold;

    int x, y = 0;

    cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(hsv[0], hsv[2], hsv[4]), cv::Scalar(hsv[1], hsv[3], hsv[5]), threshold);

    removeNoise(threshold);

    trackFilteredObject(x, y, threshold, input);

    if (x > 0 && y > 0)
    {
        double rollSpeed = (x - (threshold.cols / 2.0));
        double throttleSpeed = (y - (threshold.rows / 2.0));

        roll = this->feedbackControl.pidControl(this->targetValue, rollSpeed, this->kp, this->ki, this->kd, this->rollPrevError, this->rollIntegral, this->offset);
        throttle = this->feedbackControl.pidControl(this->targetValue, throttleSpeed, this->kp, this->ki, this->kd, this->throttlePrevError, this->throttleIntegral, this->offset);

        this->rollPrevError = this->feedbackControl.calculateError(this->targetValue, rollSpeed);
        this->throttlePrevError = this->feedbackControl.calculateError(this->targetValue, throttleSpeed);

        this->rollIntegral += this->rollPrevError;
        this->throttleIntegral += this->throttlePrevError;

        this->movement.piloting(0.0, roll, throttle);
    }

    cv::imshow(this->window, input);

    cv_ptr->image = threshold;
    cvWaitKey(50);
    imagePublisher_.publish(cv_ptr->toImageMsg());
}

/**
 * @brief The Constructor Of The Class. It Has A Base Initializer That Automatically
 *        Starts The Object Tracking.
 *
 * @param droneID string //The Drone Name(Topic Root).
 * @param hsv int[] //The HSV Color Boundaries.
 */
Tracking::Tracking(const ros::NodeHandle &nodeHandler, string droneID, string mode, int hsv[]) : nodeHandler_(nodeHandler), imageTransport_(nodeHandler_)
{
    this->movement.Setup(nodeHandler_, droneID, mode);

    std::thread t1{[&]() {
        this->movement.hover(true);
        ros::spin();
    }};

    for (int i = 0; i < 6; i++)
    {
        this->hsv[i] = hsv[i];
    }

    // Movement ps1_move(nodeHandler, droneID1, mode);
    ROS_INFO("Tracking Drone Has Begun");
    imageSubscriber_ = imageTransport_.subscribe(droneID + "/camera_iris/image_raw", 1, &Tracking::trackingCallBack, this);
    imagePublisher_ = imageTransport_.advertise(droneID + "/image_converter/output_video", 1);

    transform(droneID.begin(), droneID.end(), droneID.begin(), ::toupper);

    this->window = droneID + this->window;
    cv::namedWindow(this->window);

    t1.join();
}

/**
 * @brief The Destructor Of The Class. This Basically Stops The Object Tracking,
 *        Lands The Drone And Closes The Video Streaming Window.
 */
Tracking::~Tracking()
{
    // move.land();
    cv::destroyWindow(this->window);
}