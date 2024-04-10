/**
* @file    GoRobotPoseUtils.h
* @brief   Gocator Matrix and Pose conversion functions for different robot brands.
*
* @public
* Copyright (C) 2017-2020 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/

#ifndef GO_ROBOT_POSE_UTILS_H
#define GO_ROBOT_POSE_UTILS_H

#include <GoRobot/GoRobotApi.h>
#include <GoRobot/GoRobotPose.h>

namespace GoRobot
{
    /** Specific Pose TypeDefs for Robot Vendors **/
    typedef Pose<PoseType::XYZRPY,      AngleUnits::DEGREES, Units::MILIMETERS> KukaPose;
    typedef Pose<PoseType::ANGLE_AXIS,  AngleUnits::RADIANS, Units::METERS>     UrPose;
    typedef Pose<PoseType::XYZRPY,      AngleUnits::DEGREES, Units::MILIMETERS> YaskawaPose;

    /**
    * Creates a list of poses around an initial position
    * Can be used to scan a calibration target from multiple directions.
    * A TCP which is approximately at the target position will be created, and must be set to the robot before the poses can be used.
    *
    * @param   poseCount    The desired number of poses (including the initial pose)
    * @param   maxAngle     The maximum deviation from the Z axis in degrees.
    * @param   initialPose  The initial pose around which the ring will be created. The sensor is assumed to be looking straight down at this position.
    * @param   poseArray    A pointer to the array of points. Array should be of size poseCount.
    * @return               kStatus
    */
    GoDx(void) EyeOnHandCalibrationPoses(kSize poseCount, k64f maxAngle, Matrix initialPose, Matrix *poseArray);

    /**
    * Creates a list of poses in a single location in space
    * Can be used to scan the robot flange with a fixed mount sensor
    *
    * @param   poseCount    The desired number of poses (including the initial pose)
    * @param   initialPose  The initial pose around which the ring will be created. The sensor is assumed to be looking straight down at this position.
    * @param   poseArray    A pointer to the array of points. Array should be of size poseCount.
    * @return               kStatus
    */
    GoDx(void) EyeToHandCalibrationPoses(kSize poseCount, Matrix initialPose, Matrix *poseArray);
}
#endif // GO_ROBOT_POSE_UTILS_H
