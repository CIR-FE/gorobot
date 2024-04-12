#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "GoRobot/GoRobotMatrix.h"
#include "RobotDrivers/KukaRobotDriver.h"

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

TEST(MatrixTest, EigenvsGoRobot)
{
    double x{1053}, y{957}, z{797}, A{-163}, B{86}, C{110};

    GoRobot::Matrix goRobotMatrix = GoRobot::KukaPose(x, y, z, A, B, C).toMatrix();
    Eigen::Matrix4d eigenMatrix = xyzabcToTransformationMatrix(x, y, z, A, B, C);

    // Compare the values of goRobotMatrix and eigenMatrix
    EXPECT_DOUBLE_EQ(goRobotMatrix.Ix, eigenMatrix(0, 0));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Iy, eigenMatrix(1, 0));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Iz, eigenMatrix(2, 0));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Jx, eigenMatrix(0, 1));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Jy, eigenMatrix(1, 1));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Jz, eigenMatrix(2, 1));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Kx, eigenMatrix(0, 2));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Ky, eigenMatrix(1, 2));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Kz, eigenMatrix(2, 2));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Tx, eigenMatrix(0, 3));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Ty, eigenMatrix(1, 3));
    EXPECT_DOUBLE_EQ(goRobotMatrix.Tz, eigenMatrix(2, 3));
}

TEST(MatrixTest, Multiplication)
{
    // Arrange
    GoRobot::Matrix matrix1(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                            0.0, 0.0, 0.0);

    GoRobot::Matrix matrix2(2.0, 0.0, 0.0,
                            0.0, 2.0, 0.0,
                            0.0, 0.0, 2.0,
                            0.0, 0.0, 0.0);

    // Act
    GoRobot::Matrix result = matrix1 * matrix2;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, 2.0);
    EXPECT_DOUBLE_EQ(result.Iy, 0.0);
    EXPECT_DOUBLE_EQ(result.Iz, 0.0);
    EXPECT_DOUBLE_EQ(result.Jx, 0.0);
    EXPECT_DOUBLE_EQ(result.Jy, 2.0);
    EXPECT_DOUBLE_EQ(result.Jz, 0.0);
    EXPECT_DOUBLE_EQ(result.Kx, 0.0);
    EXPECT_DOUBLE_EQ(result.Ky, 0.0);
    EXPECT_DOUBLE_EQ(result.Kz, 2.0);
    EXPECT_DOUBLE_EQ(result.Tx, 0.0);
    EXPECT_DOUBLE_EQ(result.Ty, 0.0);
    EXPECT_DOUBLE_EQ(result.Tz, 0.0);
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

TEST(MatrixTest, KukaPose)
{
    // Arrange
    GoRobot::Matrix matrix = GoRobot::KukaPose(1054, 957, 797, 110, 86, -163).toMatrix();
    GoRobot::Matrix movePose = GoRobot::KukaPose(0, 0, -200, 0, 0, 0).toMatrix();

    // Act
    GoRobot::Matrix result = matrix * movePose;

    // Assert
    EXPECT_DOUBLE_EQ(result.Ix, -0.066708447600717785);
    EXPECT_DOUBLE_EQ(result.Iy, -0.020394819144016807);
    EXPECT_DOUBLE_EQ(result.Iz, -0.99756405025982420);
    EXPECT_DOUBLE_EQ(result.Jx, -0.99644051109511778);
    EXPECT_DOUBLE_EQ(result.Jy, 0.053005207938078114);
    EXPECT_DOUBLE_EQ(result.Jz, 0.065549643629400661);
    EXPECT_DOUBLE_EQ(result.Kx, 0.051539216788796971);
    EXPECT_DOUBLE_EQ(result.Ky, 0.99838594705831274);
    EXPECT_DOUBLE_EQ(result.Kz, -0.023858119147859035);
    EXPECT_DOUBLE_EQ(result.Tx, 1054.0);
    EXPECT_DOUBLE_EQ(result.Ty, 757.0);
    EXPECT_DOUBLE_EQ(result.Tz, 597.0);
}
