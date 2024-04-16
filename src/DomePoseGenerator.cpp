#include "DomePoseGenerator.h"
#include <numbers>
#include <cmath>

// Function to generate dome-like pattern of poses
std::vector<GoRobot::Matrix> DomePoseGenerator::generateDomePoses(const GoRobot::Matrix &initPose, int numPoses, double maxXDegrees, double maxYDegrees, double maxZDegrees)
{
    Eigen::Matrix4d initPoseEigen = convertToEigenMatrix(initPose);
    std::vector<GoRobot::Matrix> poses;
    poses.push_back(initPose);

    double thetaIncrement = 2 * maxXDegrees * std::numbers::pi / (180.0 * numPoses);
    double phiIncrement = 2 * maxYDegrees * std::numbers::pi / (180.0 * numPoses);
    double psiIncrement = 2 * maxZDegrees * std::numbers::pi / (180.0 * numPoses);

    // Generate the poses
    for (int i = 0; i < numPoses; ++i)
    {
        double theta = -maxXDegrees * std::numbers::pi / 180.0 + i * thetaIncrement;
        double phi = -maxYDegrees * std::numbers::pi / 180.0 + i * phiIncrement;
        double psi = -maxZDegrees * std::numbers::pi / 180.0 + i * psiIncrement;

        // Convert the spherical coordinates to Cartesian coordinates
        double x = std::sin(phi) * std::cos(theta);
        double y = std::sin(phi) * std::sin(theta);
        double z = std::cos(phi);

        // Create a translation
        Eigen::Translation3d translation(x, y, z);

        // Create a rotation
        Eigen::Matrix4d rotation4d = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation1(theta, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation2(phi, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation3(psi, Eigen::Vector3d::UnitZ());
        rotation4d.block<3, 3>(0, 0) = (rotation1 * rotation2 * rotation3).matrix();

        // Convert the 3D translation to a 4x4 transformation matrix
        Eigen::Matrix4d translation4d = Eigen::Matrix4d::Identity();
        translation4d.block<3, 1>(0, 3) = translation.vector();

        // Combine the translation and rotation into a transformation matrix
        Eigen::Matrix4d pose = initPoseEigen * rotation4d * translation4d;

        // Add the pose to the list
        poses.push_back(convertToGoRobotMatrix(pose));
    }

    return poses;
}

namespace
{
    Eigen::Matrix4d convertToEigenMatrix(const GoRobot::Matrix &goMatrix)
    {
        Eigen::Matrix4d eigenMatrix;

        eigenMatrix << goMatrix.Ix, goMatrix.Jx, goMatrix.Kx, goMatrix.Tx,
            goMatrix.Iy, goMatrix.Jy, goMatrix.Ky, goMatrix.Ty,
            goMatrix.Iz, goMatrix.Jz, goMatrix.Kz, goMatrix.Tz,
            0, 0, 0, 1;

        return eigenMatrix;
    }

    GoRobot::Matrix convertToGoRobotMatrix(const Eigen::Matrix4d &eigenMatrix)
    {
        GoRobot::Matrix goMatrix;

        goMatrix.Ix = eigenMatrix(0, 0);
        goMatrix.Jx = eigenMatrix(0, 1);
        goMatrix.Kx = eigenMatrix(0, 2);
        goMatrix.Tx = eigenMatrix(0, 3);

        goMatrix.Iy = eigenMatrix(1, 0);
        goMatrix.Jy = eigenMatrix(1, 1);
        goMatrix.Ky = eigenMatrix(1, 2);
        goMatrix.Ty = eigenMatrix(1, 3);

        goMatrix.Iz = eigenMatrix(2, 0);
        goMatrix.Jz = eigenMatrix(2, 1);
        goMatrix.Kz = eigenMatrix(2, 2);
        goMatrix.Tz = eigenMatrix(2, 3);

        return goMatrix;
    }
}