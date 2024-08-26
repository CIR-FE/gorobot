/**
 * @file    GoRobotMatrix.h
 * @brief   Gocator Matrix definitions.
 *
 * @public
 * Copyright (C) 2017-2020 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_ROBOT_MATRIX_H
#define GO_ROBOT_MATRIX_H

#include <GoRobot/GoRobotApi.h>

namespace GoRobot
{
/**
 * @class   Matrix
 * @brief   A 6 degrees of freedom transformation matrix in 3D space     <br>
 *          Units are in milimeters                                      <br>
 *           [ Ix Jx Kx Tx ]                                             <br>
 *           [ Iy Jy Ky Ty ]                                             <br>
 *           [ Iz Jz Kz Tz ]                                             <br>
 *           [ 0  0  0  1  ]
 */
class GoDx(Matrix)
{
 public:
    double Ix, Iy, Iz;
    double Jx, Jy, Jz;
    double Kx, Ky, Kz;
    double Tx, Ty, Tz;

    Matrix();
    Matrix(double Ix, double Iy, double Iz, double Jx, double Jy, double Jz, double Kx, double Ky,
           double Kz, double Tx, double Ty, double Tz);
    Matrix(kPoint3d64f I, kPoint3d64f J, kPoint3d64f K, kPoint3d64f T);

    /* Reset the matrix to a unity matrix */
    void Unity();

    /* Check whether this matrix is a unity matrix */
    bool isUnity() const;
    Matrix Rotate(k64f degrees, kPoint3d64f axis) const;
    Matrix Translate(kPoint3d64f t) const;
    Matrix Inverse() const;
    Matrix operator*(Matrix other) const;
};

/** Unity Matrix */
static const Matrix MATRIX_UNITY{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
}  // namespace GoRobot
#endif  // !GO_ROBOT_MATRIX_H
