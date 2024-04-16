#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "GoRobot/GoRobotMatrix.h"
#include "RobotDrivers/KukaRobotDriver.h"
#include "DomePoseGenerator.h"

Eigen::Matrix4d xyzabcToTransformationMatrix(double x, double y, double z, double a, double b, double c)
{
    // Convert a, b, c to a rotation matrix
    Eigen::AngleAxisd aAngle(a, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd bAngle(b, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd cAngle(c, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = cAngle * bAngle * aAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    // Create a 4x4 transformation matrix
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

    // Set the rotation part
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;

    // Set the translation part
    transformationMatrix(0, 3) = x;
    transformationMatrix(1, 3) = y;
    transformationMatrix(2, 3) = z;

    return transformationMatrix;
}

TEST(MatrixTest, Dome)
{
    const size_t numberPoses{5};
    const size_t maxDeviationDegrees{7};
    // std::vector<GoRobot::Matrix> tcpPoses(numberPoses);
    // std::vector<GoRobot::KukaPose> goRobotKukaPoses(numberPoses);
    std::vector<GoRobot::KukaPose> eigenKukaPoses;
    GoRobot::Matrix initPose = GoRobot::KukaPose(991, 978, 488, 180, 0, -90).toMatrix(); // X 996, Y 1061, Z 785, A -90, B 0, C 180

    // GoRobot::EyeOnHandCalibrationPoses(numberPoses, maxDeviationDegrees, initPose, tcpPoses.data());

    std::vector<GoRobot::Matrix> eigenPoses = DomePoseGenerator::generateDomePoses(initPose, numberPoses, 15, 15, 5);

    for (const auto &pose : eigenPoses)
    {
        // goRobotKukaPoses[i] = GoRobot::KukaPose(tcpPoses[i]);
        eigenKukaPoses.push_back(GoRobot::KukaPose(pose));
    }
}

TEST(MatrixTest, Multiplication)
{
    // Arrange
    GoRobot::Matrix matrix1(2.0, 3.0, 4.0,
                            5.0, 6.0, 7.0,
                            8.0, 9.0, 10.0,
                            11.0, 12.0, 13.0);

    GoRobot::Matrix matrix2(2.0, 0.0, 0.0,
                            0.0, 2.0, 0.0,
                            0.0, 0.0, 2.0,
                            5.0, 6.0, 7.0);

    // Act
    GoRobot::Matrix result = matrix1 * matrix2;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, 4.0);
    EXPECT_DOUBLE_EQ(result.Iy, 6.0);
    EXPECT_DOUBLE_EQ(result.Iz, 8.0);
    EXPECT_DOUBLE_EQ(result.Jx, 10.0);
    EXPECT_DOUBLE_EQ(result.Jy, 12.0);
    EXPECT_DOUBLE_EQ(result.Jz, 14.0);
    EXPECT_DOUBLE_EQ(result.Kx, 16.0);
    EXPECT_DOUBLE_EQ(result.Ky, 18.0);
    EXPECT_DOUBLE_EQ(result.Kz, 20.0);
    EXPECT_DOUBLE_EQ(result.Tx, 107.0);
    EXPECT_DOUBLE_EQ(result.Ty, 126.0);
    EXPECT_DOUBLE_EQ(result.Tz, 145.0);
}

TEST(MatrixTest, OnlyTranslation)
{
    // Arrange
    GoRobot::Matrix matrix1(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                            11.0, 12.0, 13.0);

    GoRobot::Matrix matrix2(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                            5.0, 6.0, 7.0);

    // Act
    GoRobot::Matrix result = matrix1 * matrix2;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, 1.0);
    EXPECT_DOUBLE_EQ(result.Iy, 0.0);
    EXPECT_DOUBLE_EQ(result.Iz, 0.0);
    EXPECT_DOUBLE_EQ(result.Jx, 0.0);
    EXPECT_DOUBLE_EQ(result.Jy, 1.0);
    EXPECT_DOUBLE_EQ(result.Jz, 0.0);
    EXPECT_DOUBLE_EQ(result.Kx, 0.0);
    EXPECT_DOUBLE_EQ(result.Ky, 0.0);
    EXPECT_DOUBLE_EQ(result.Kz, 1.0);
    EXPECT_DOUBLE_EQ(result.Tx, 16.0);
    EXPECT_DOUBLE_EQ(result.Ty, 18.0);
    EXPECT_DOUBLE_EQ(result.Tz, 20.0);
}

TEST(MatrixTest, KukaToMatrix)
{
    // Arrange
    GoRobot::Matrix matrix1 = GoRobot::KukaPose(996, 1061, 785, 0, 0, 0).toMatrix();

    GoRobot::Matrix matrix2(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                            996.0, 1061.0, 785.0);

    // Assert
    EXPECT_DOUBLE_EQ(matrix1.Ix, matrix2.Ix);
    EXPECT_DOUBLE_EQ(matrix1.Iy, matrix2.Iy);
    EXPECT_DOUBLE_EQ(matrix1.Iz, matrix2.Iz);
    EXPECT_DOUBLE_EQ(matrix1.Jx, matrix2.Jx);
    EXPECT_DOUBLE_EQ(matrix1.Jy, matrix2.Jy);
    EXPECT_DOUBLE_EQ(matrix1.Jz, matrix2.Jz);
    EXPECT_DOUBLE_EQ(matrix1.Kx, matrix2.Kx);
    EXPECT_DOUBLE_EQ(matrix1.Ky, matrix2.Ky);
    EXPECT_DOUBLE_EQ(matrix1.Kz, matrix2.Kz);
    EXPECT_DOUBLE_EQ(matrix1.Tx, matrix2.Tx);
    EXPECT_DOUBLE_EQ(matrix1.Ty, matrix2.Ty);
    EXPECT_DOUBLE_EQ(matrix1.Tz, matrix2.Tz);
}

TEST(MatrixTest, KukaTranslation)
{
    // Arrange
    GoRobot::Matrix matrix = GoRobot::KukaPose(100, 200, 300, 0, 0, 0).toMatrix();
    GoRobot::Matrix movePose = GoRobot::KukaPose(0, 0, 200, 0, 0, 0).toMatrix();

    // Act
    GoRobot::Matrix result = matrix * movePose;
    GoRobot::KukaPose kukaPose = GoRobot::KukaPose(result);

    // Assert
    EXPECT_DOUBLE_EQ(kukaPose.x, 100.0);
    EXPECT_DOUBLE_EQ(kukaPose.y, 200.0);
    EXPECT_DOUBLE_EQ(kukaPose.z, 500.0);
    EXPECT_DOUBLE_EQ(kukaPose.rx, 0.0);
    EXPECT_DOUBLE_EQ(kukaPose.ry, 0.0);
    EXPECT_DOUBLE_EQ(kukaPose.rz, 0.0);
}

TEST(MatrixTest, KukaTranslation2)
{
    // Arrange
    GoRobot::Matrix initPose = GoRobot::KukaPose(991, 978, 488, 180, 0, -90).toMatrix(); // X 996, Y 1061, Z 785, A -90, B 0, C 180
    GoRobot::Matrix movePose = GoRobot::KukaPose(200, 0, 0, 0, 0, 0).toMatrix();

    // Act
    GoRobot::Matrix result = initPose * movePose;
    GoRobot::KukaPose kukaPose = GoRobot::KukaPose(result);

    // Assert
    EXPECT_DOUBLE_EQ(kukaPose.x, 991.0);
    EXPECT_DOUBLE_EQ(kukaPose.y, 778.0);
    EXPECT_DOUBLE_EQ(kukaPose.z, 488.0);
    EXPECT_DOUBLE_EQ(kukaPose.rx, 180.0);
    EXPECT_DOUBLE_EQ(kukaPose.ry, 0.0);
    EXPECT_DOUBLE_EQ(kukaPose.rz, -90.0);
}

TEST(MatrixTest, Inverse)
{
    // Arrange
    GoRobot::Matrix initPose = GoRobot::KukaPose(996, 1061, 785, 90, 0, 0).toMatrix();
    GoRobot::Matrix endPose = GoRobot::KukaPose(996, 861, 785, 90, 0, 0).toMatrix();

    GoRobot::Matrix expected = GoRobot::KukaPose(0, -200, 0, 0, 0, 0).toMatrix();

    // Act
    GoRobot::Matrix result = endPose * initPose.Inverse();

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, expected.Ix);
    EXPECT_DOUBLE_EQ(result.Iy, expected.Iy);
    EXPECT_DOUBLE_EQ(result.Iz, expected.Iz);
    EXPECT_DOUBLE_EQ(result.Jx, expected.Jx);
    EXPECT_DOUBLE_EQ(result.Jy, expected.Jy);
    EXPECT_DOUBLE_EQ(result.Jz, expected.Jz);
    EXPECT_DOUBLE_EQ(result.Kx, expected.Kx);
    EXPECT_DOUBLE_EQ(result.Ky, expected.Ky);
    EXPECT_DOUBLE_EQ(result.Kz, expected.Kz);
    EXPECT_DOUBLE_EQ(result.Tx, expected.Tx);
    EXPECT_DOUBLE_EQ(result.Ty, expected.Ty);
    EXPECT_DOUBLE_EQ(result.Tz, expected.Tz);
}

TEST(MatrixTest, IdentityMultiplication)
{
    // Arrange
    GoRobot::Matrix matrix(2.0, 3.0, 4.0,
                           5.0, 6.0, 7.0,
                           8.0, 9.0, 10.0,
                           11.0, 12.0, 13.0);

    GoRobot::Matrix identity(1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0,
                             0.0, 0.0, 0.0);

    // Act
    GoRobot::Matrix result = matrix * identity;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, matrix.Ix);
    EXPECT_DOUBLE_EQ(result.Iy, matrix.Iy);
    EXPECT_DOUBLE_EQ(result.Iz, matrix.Iz);
    EXPECT_DOUBLE_EQ(result.Jx, matrix.Jx);
    EXPECT_DOUBLE_EQ(result.Jy, matrix.Jy);
    EXPECT_DOUBLE_EQ(result.Jz, matrix.Jz);
    EXPECT_DOUBLE_EQ(result.Kx, matrix.Kx);
    EXPECT_DOUBLE_EQ(result.Ky, matrix.Ky);
    EXPECT_DOUBLE_EQ(result.Kz, matrix.Kz);
    EXPECT_DOUBLE_EQ(result.Tx, matrix.Tx);
    EXPECT_DOUBLE_EQ(result.Ty, matrix.Ty);
    EXPECT_DOUBLE_EQ(result.Tz, matrix.Tz);
}

TEST(MatrixTest, ZeroMultiplication)
{
    // Arrange
    GoRobot::Matrix matrix(2.0, 3.0, 4.0,
                           5.0, 6.0, 7.0,
                           8.0, 9.0, 10.0,
                           11.0, 12.0, 13.0);

    GoRobot::Matrix zero(0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0);

    // Act
    GoRobot::Matrix result = matrix * zero;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, 0.0);
    EXPECT_DOUBLE_EQ(result.Iy, 0.0);
    EXPECT_DOUBLE_EQ(result.Iz, 0.0);
    EXPECT_DOUBLE_EQ(result.Jx, 0.0);
    EXPECT_DOUBLE_EQ(result.Jy, 0.0);
    EXPECT_DOUBLE_EQ(result.Jz, 0.0);
    EXPECT_DOUBLE_EQ(result.Kx, 0.0);
    EXPECT_DOUBLE_EQ(result.Ky, 0.0);
    EXPECT_DOUBLE_EQ(result.Kz, 0.0);
    EXPECT_DOUBLE_EQ(result.Tx, 11.0);
    EXPECT_DOUBLE_EQ(result.Ty, 12.0);
    EXPECT_DOUBLE_EQ(result.Tz, 13.0);
}

// Test the Unity method
TEST(MatrixTest, Unity)
{
    // Arrange
    GoRobot::Matrix matrix(2.0, 3.0, 4.0,
                           5.0, 6.0, 7.0,
                           8.0, 9.0, 10.0,
                           11.0, 12.0, 13.0);

    // Act
    matrix.Unity();

    // Assert
    EXPECT_DOUBLE_EQ(matrix.Ix, 1.0);
    EXPECT_DOUBLE_EQ(matrix.Iy, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Iz, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Jx, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Jy, 1.0);
    EXPECT_DOUBLE_EQ(matrix.Jz, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Kx, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Ky, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Kz, 1.0);
    EXPECT_DOUBLE_EQ(matrix.Tx, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Ty, 0.0);
    EXPECT_DOUBLE_EQ(matrix.Tz, 0.0);
}

// Test the isUnity method
TEST(MatrixTest, isUnity)
{
    // Arrange
    GoRobot::Matrix matrix;

    // Act & Assert
    EXPECT_TRUE(matrix.isUnity());

    // Arrange
    matrix = GoRobot::Matrix(2.0, 3.0, 4.0,
                             5.0, 6.0, 7.0,
                             8.0, 9.0, 10.0,
                             11.0, 12.0, 13.0);

    // Act & Assert
    EXPECT_FALSE(matrix.isUnity());
}
