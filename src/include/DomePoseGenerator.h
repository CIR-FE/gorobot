#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GoRobot/GoRobotMatrix.h>

namespace
{
    Eigen::Matrix4d convertToEigenMatrix(const GoRobot::Matrix &goMatrix);
    GoRobot::Matrix convertToGoRobotMatrix(const Eigen::Matrix4d &eigenMatrix);
}

namespace DomePoseGenerator
{
    std::vector<GoRobot::Matrix> generateDomePoses(const GoRobot::Matrix &initPose, int numPoses, double maxXDegrees, double maxYDegrees, double maxZDegrees);
}
