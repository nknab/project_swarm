/**
 * @author: nknab
 * @date: Monday, 10th December 2018.
 * @time: 9:46 PM
 * @project: Project Swarm
 * @version: 1.0
 * @brief: This Class Is Responsible For The Closed Loop Control.
 */

//Import Of Class Header File.
#include "FeedbackControl.h"

/**
 * @brief The Constructor Of The Class.
 */
FeedbackControl::FeedbackControl() = default;

/**
 * @brief The Destructor Of The Class.
 */
FeedbackControl::~FeedbackControl() = default;

/**
 * @brief This Calculates The Error Between The Target Value And The Value Obtained.
 *
 * @param targetValue double //The Target Value of the System.
 * @param value double //The Value Obtained From The System.
 *
 * @return double //The Error.
 */
double FeedbackControl::calculateError(double targetValue, double value) {
    double error = targetValue - value;

    return error;
}

/**
 * @brief This Is A Proportional Control Function.
 *
 * @param targetValue double //The Target Value of the System.
 * @param value double //The Value Obtained From The System.
 * @param kp double //The Proportional Gain Constant.
 *
 * @return double //The P Value.
 */
double FeedbackControl::pControl(double targetValue, double value, double kp) {
    double error = calculateError(targetValue, value);
    double p = error * kp;

    return p;
}

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
double FeedbackControl::pdControl(double targetValue, double value, double kp, double kd, double previousError) {
    double p = pControl(targetValue, value, kp);
    double error = calculateError(targetValue, value);
    double derivative = error - previousError;
    double d = derivative * kd;
    double pd = p + d;

    return pd;
}

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
double FeedbackControl::pidControl(double targetValue, double value, double kp, double ki, double kd, double previousError, double integral, double offset) {
    double pd = pdControl(targetValue, value, kp, kd, previousError);
    double i = integral * ki;
    double pid = pd + i + offset;

    return pid;
}