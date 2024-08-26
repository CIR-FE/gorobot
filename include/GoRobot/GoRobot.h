/**
 * @file    GoRobot.h
 * @brief   Gocator sensor to Robot calibration functions library.
 *
 * @public
 * Copyright (C) 2017-2020 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_ROBOT_H
#define GO_ROBOT_H

#include <GoRobot/GoRobotApi.h>
#include <GoRobot/GoRobotMatrix.h>
#include <GoRobot/GoRobotPose.h>
#include <GoRobot/GoRobotPoseUtils.h>
#include <GoSdk/GoSensor.h>

namespace GoRobot
{
/**
 * @struct  Accuracy
 * @brief   Metrics to quantify the calibration accuracy
 *
 * Including values of pose averages, max minus min ranges and standard deviations
 */
struct Accuracy {
    Matrix average;
    k64f angleRange;
    k64f angleStd;
    k64f positionRange;
    k64f positionStd;
};

/**
 * Sets up the calibration job, with multiple exposures,
 * Software trigger (for G3) or Time mode (for G2), and the SurfaceBallBar with all
 * outputs selected, with ids 0-11, and outputed using Gocator protocol
 *
 * @param   sensor          The sensor to be set up
 */
GoDx(void) SetupCalibrationJob(GoSensor sensor);

/**
 * Get a pose from the SurfaceBallBar tool,
 * Run through the given dataset and looks for a matrix as 12 measurements
 * The matrix should have consecutive ID numbers.
 *
 * @param   dataset      The dataset received from a trigger of the sensor
 * @param   id           The id of the first matrix value (Ix)
 * @return               Matrix output
 */
GoDx(Matrix) GetBallBarPoseMeasurement(GoDataSet dataset, kSize id);

/**
 * For a sensor mounted on the robot arm.
 * Given the data from robot and sensor, calibrate the Eye-on-Hand system.
 *
 * @param   arr_bf       (f)lange poses in robot (b)ase
 * @param   arr_st       (t)arget pose in (s)ensor
 * @param   count        Number of pose sets given
 * @return               Calibrated hand eye transform matrix
 */
GoDx(Matrix) CalibEyeOnHandSystem(const Matrix* arr_bf, const Matrix* arr_st, k32s count);

/**
 * For a sensor that is fixed mounted above the working area.
 * Given the data from robot and sensor, calibrate the Eye-to-Hand system.
 *
 * @param   arr_bf       (f)lange poses in robot (b)ase
 * @param   arr_st       (t)arget pose in (s)ensor
 * @param   count        Number of pose sets given
 * @return               Calibrated hand eye transform matrix
 */
GoDx(Matrix) CalibEyeToHandSystem(const Matrix* arr_bf, const Matrix* arr_st, k32s count);

/**
 * For a sensor mounted on the robot arm.
 * Given the data from robot and sensor, use the calibrated hand-eye transform to compute
 * the object pose in the robot base coordinate system.
 *
 * @param   arr_bf              (f)lange poses in robot (b)ase coordinate system
 * @param   arr_st              (t)arget pose in (s)ensor coordinate system
 * @param   count               Number of pose sets given
 * @param   handEyeMat          Calibrated hand eye transformation matrix
 * @param   arr_bt              (t)arget pose in robot (b)ase
 */
GoDx(void) LocateEyeOnHandSystem(const Matrix* arr_bf, const Matrix* arr_st, k32s count,
                                 Matrix handEyeMat, Matrix* arr_bt);

/**
 * For a sensor that is fixed mounted above the working area.
 * Given the data from sensor, use the calibrated hand-eye transform to compute
 * the object pose in the robot base coordinate system.
 *
 * @param   arr_st              (t)arget pose in (s)ensor coordinate system
 * @param   count               Number of pose sets given
 * @param   handEyeMat          Calibrated hand eye transformation matrix
 * @param   arr_bt              (t)arget pose in robot (b)ase
 */
GoDx(void)
    LocateEyeToHandSystem(const Matrix* arr_st, k32s count, Matrix handEyeMat, Matrix* arr_bt);

/**
 * Analyse the accuracy of the calibration routine
 * @param   arr_bt       (t)arget pose in robot (b)ase
 * @param   count        Number of pose sets given
 * @return               Calibration accuracy metrics
 */
GoDx(Accuracy) MeasureAccuracy(const Matrix* arr_bt, k32s count);
}  // namespace GoRobot
#endif  // GO_ROBOT_H
