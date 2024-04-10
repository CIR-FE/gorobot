/**
* @file    GoRobotPose.h
* @brief   Gocator Pose definitions.
*
* @public
* Copyright (C) 2017-2020 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/

#ifndef GO_ROBOT_POSE_H
#define GO_ROBOT_POSE_H

#include <GoRobot/GoRobotApi.h>
#include <GoRobot/GoRobotMatrix.h>

namespace GoRobot
{
    enum class PoseType
    {
        XYZRPY,
        ANGLE_AXIS
    };
    enum class Units
    {
        METERS,
        MILIMETERS
    };
    enum class AngleUnits
    {
        DEGREES,
        RADIANS
    };

    GoDx(void)    PoseValuesFromMatrix(const Matrix& matrix,
        PoseType pose_type, AngleUnits angle_units, Units units,
        double& x, double& y, double& z, double& rx, double& ry, double& rz);
    GoDx(Matrix)  PoseValuesToMatrix(
        PoseType pose_type, AngleUnits angle_units, Units units,
        double x, double y, double z, double rx, double ry, double rz);
    
    /**
    * @class   Pose
    * @brief   A 3 axes pose in 3D space
    *          Could be represented in numerous formats:
    *          (XYZRPY, AxisAngle, etc.)
    *          See GoRobotPoseUtils.h for conversions
    */
    template<PoseType pose_type = PoseType::XYZRPY, AngleUnits angle_units = AngleUnits::DEGREES, Units units = Units::MILIMETERS>
    class Pose
    {
    private:
    public:
        double x, y, z, rx, ry, rz;

        Pose() :
            x(0), y(0), z(0), rx(0), ry(0), rz(0)
        {        }
        Pose(double x, double y, double z, double rx, double ry, double rz) :
            x(x), y(y), z(z), rx(rx), ry(ry), rz(rz)
        {        }
        Pose(const Matrix& matrix)
        {
            PoseValuesFromMatrix(matrix, pose_type, angle_units, units, x, y, z, rx, ry, rz);
        }

        Matrix toMatrix()
        {
            return PoseValuesToMatrix(pose_type, angle_units, units, x, y, z, rx, ry, rz);
        }
    };
}
#endif // !GO_ROBOT_POSE_H
