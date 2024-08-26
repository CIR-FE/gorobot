/**
 * @file    GoRobotDriverInterface.h
 * @brief   Robot Driver Interface to be used with the GoRobot library.
 *
 * @public
 * Copyright (C) 2017-2020 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_ROBOT_DRIVER_INTERFACE_H
#define GO_ROBOT_DRIVER_INTERFACE_H

#include <GoRobot/GoRobotMatrix.h>

namespace GoRobot
{
/**
 * @enum     MoveMode
 * @brief    Robot arm can be moved in different modes,
 *           this enum distinguishes between them.
 *           More modes might be added in the future.
 */
enum class MoveMode {
    LINEAR,
    CIRCLE,
    CONTINOUS_LINEAR,
    CONTINOUS_CIRCLE
};

/**
 * @interface    Driver
 * @brief        A robot driver must implement at least the functions below.
 */
class Driver
{
 public:
    /**
     * Move the arm through the list of points provided, using the mode specified
     *
     * @param   poseList     A pointer to the beginning of the list
     * @param   poseCount    The number of poses in the list
     * @param   moveMode     Specifies the movement method
     */
    virtual void move(GoRobot::Matrix* poseList, size_t poseCount = 1,
                      GoRobot::MoveMode moveMode = GoRobot::MoveMode::LINEAR) = 0;

    /**
     * Sets the robot's base coordinate system
     *
     * @param   base     The base to set
     */
    virtual void set_base(GoRobot::Matrix base) = 0;

    /**
     * Sets the robot's acceleration and moving speed
     *
     * @param   acceleration     The new acceleration to set in mm/sec^2
     * @param   speed            The new speed to set in mm/sec
     * @param   moveMode         Specifies the movement method
     */
    virtual void set_acc_speed(double acceleration, double speed,
                               GoRobot::MoveMode moveMode = GoRobot::MoveMode::LINEAR) = 0;

    /**
     * Sets the robot's tool center point (tcp)
     *
     * @param   tcp     The tcp to set
     */
    virtual void set_tcp(GoRobot::Matrix tcp) = 0;

    /**
     * Get the current position of the tool
     *
     * @return   The current position of the tool in matrix form
     */
    virtual GoRobot::Matrix get_tcp_pose() = 0;

    /**
     * Get the current position of the flange
     *
     * @return   The current position of the flange in matrix form
     */
    virtual GoRobot::Matrix get_flange_pose() = 0;
};
}  // namespace GoRobot
#endif  // GO_ROBOT_DRIVER_INTERFACE_H