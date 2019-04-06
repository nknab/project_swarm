/*
 * File: FeedbackControl.h
 * Project: Project Swarm PX4
 * File Created: Saturday, 6th April 2019 1:10:46 AM
 * Author: nknab
 * Email: bkojo21@gmail.com
 * Version: 1.0
 * Brief: 
 * -----
 * Last Modified: Saturday, 6th April 2019 1:10:48 AM
 * Modified By: nknab
 * -----
 * Copyright ©2019 nknab
 */

#ifndef PROJECT_FEEDBACKCONTROL_H
#define PROJECT_FEEDBACKCONTROL_H

class FeedbackControl
{

  public:
    /**
     * @brief The Constructor Of The Class.
     */
    explicit FeedbackControl();

    /**
     * @brief The Destructor Of The Class.
     */
    virtual ~FeedbackControl();

    /**
     * @brief This Calculates The Error Between The Target Value And The Value Obtained.
     *
     * @param targetValue double //The Target Value of the System.
     * @param value double //The Value Obtained From The System.
     *
     * @return double //The Error.
     */
    double calculateError(double targetValue, double value);

    /**
     * @brief This Is A Proportional Control Function.
     *
     * @param targetValue double //The Target Value of the System.
     * @param value double //The Value Obtained From The System.
     * @param kp double //The Proportional Gain Constant.
     *
     * @return double //The P Value.
     */
    double pControl(double targetValue, double value, double kp);

    /**
     * @brief This Is A Proportional Derivative Control Function.
     *
     * @param targetValue double //The Target Value of the System.
     * @param value double //The Value Obtained From The System.
     * @param kp double //The Proportional Gain Constant.
     * @param kd double //The Derivative Gain Constant.
     * @param previousError double //The Previous Error Obtained from The System.
     *
     * @return double //The PD Value.
     */
    double pdControl(double targetValue, double value, double kp, double kd, double previousError);

    /**
     * @brief This Is A Proportional Integral Derivative Control Function.
     *
     * @param targetValue double //The Target Value of the System.
     * @param value double //The Value Obtained From The System.
     * @param kp double //Proportional Gain Constant.
     * @param ki double //The Integral Gain Constant.
     * @param kd double //The Derivative Gain Constant.
     * @param previousError double //The Previous Error Obtained from The System.
     * @param integral double //The Sum Of All Errors Obtained From The System.
     * @param offset double //The Default Value Of The System If There is No Error.
     *
     * @return double //The PID Value.
     */
    double pidControl(double targetValue, double value, double kp, double ki, double kd, double previousError, double integral, double offset);
};

#endif //PROJECT_FEEDBACKCONTROL_H