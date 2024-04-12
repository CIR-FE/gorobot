#include <gtest/gtest.h>
#include "GoRobot/GoRobotMatrix.h"
#include "RobotDrivers/KukaRobotDriver.h"

void printMatrix(const GoRobot::Matrix &m, bool multiline = false)
{
    if (multiline)
    {
        printf("%12.3e, %12.3e, %12.3e, %12.3e, \n", m.Ix, m.Jx, m.Kx, m.Tx);
        printf("%12.3e, %12.3e, %12.3e, %12.3e, \n", m.Iy, m.Jy, m.Ky, m.Ty);
        printf("%12.3e, %12.3e, %12.3e, %12.3e, \n", m.Iz, m.Jz, m.Kz, m.Tz);
        printf("%12.3e, %12.3e, %12.3e, %12.3e, \n", 0.0, 0.0, 0.0, 1.0);
    }
    else
    {
        printf("%12.3e, %12.3e, %12.3e, ", m.Ix, m.Iy, m.Iz);
        printf("%12.3e, %12.3e, %12.3e, ", m.Jx, m.Jy, m.Jz);
        printf("%12.3e, %12.3e, %12.3e, ", m.Kx, m.Ky, m.Kz);
        printf("%12.3e, %12.3e, %12.3e \n", m.Tx, m.Ty, m.Tz);
    }
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
    GoRobot::Matrix movePose = GoRobot::KukaPose(0, -200, 0, 0, 0, 0).toMatrix();

    printMatrix(matrix, true);
    printf("\n");
    printMatrix(movePose, true);
    printf("\n");

    // Act
    GoRobot::Matrix result = matrix * movePose;

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
    EXPECT_DOUBLE_EQ(result.Tx, 1054.0);
    EXPECT_DOUBLE_EQ(result.Ty, 757.0);
    EXPECT_DOUBLE_EQ(result.Tz, 797.0);
}
