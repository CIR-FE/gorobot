/**
 * KukaHandEyeCalibration.cpp
 * An example to doing a hand-eye calibration on a Kuka Robot
 *
 * @public
 * Copyright (C) 2017-2018 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include <GoRobot/GoRobot.h>
#include <GoSdk/GoSdk.h>
#include <RobotDrivers/KukaRobotDriver.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <windows.h>

#include <iostream>
#include <string>
#include <vector>

#define SENSOR_IP_ADDRESS "192.168.1.10"  // Change this to the Sensor's IP address
#define ROBOT_IP_ADDRESS "192.168.1.20"   // Change this to the Robot's IP address
#define ROBOT_PORT \
    54600  // Change this to the Robot's Connection Port (specified in the
           // XmlServerFollow.xml)

#define NUMBER_OF_POSES 5
#define SPEED 100                 // mm/sec
#define ACCELERATION 150000       // mm/sec^2
#define SURFACE_LENGTH 150        // mm
#define SURFACE_LENGTH_BUFFER 50  // mm

std::string formatMatrix(const std::string &name, const GoRobot::Matrix &m)
{
    return fmt::format(
        "{}:\n{:12.3e}, {:12.3e}, {:12.3e}, {:12.3e},\n{:12.3e}, {:12.3e}, {:12.3e}, "
        "{:12.3e},\n{:12.3e}, {:12.3e}, {:12.3e}, {:12.3e},\n{:12.3e}, {:12.3e}, {:12.3e}, "
        "{:12.3e}",
        name, m.Ix, m.Jx, m.Kx, m.Tx, m.Iy, m.Jy, m.Ky, m.Ty, m.Iz, m.Jz, m.Kz, m.Tz, 0.0, 0.0, 0.0,
        1.0);
}

GoRobot::Matrix initPose = GoRobot::KukaPose(250, -120, 30, 180, 0, 0).toMatrix();
GoRobot::Matrix movePose =
    GoRobot::KukaPose(0, -(SURFACE_LENGTH + SURFACE_LENGTH_BUFFER), 0, 0, 0, 0).toMatrix();
GoRobot::Matrix tcp = GoRobot::KukaPose(0, 0, 200, 0, 0, 0).toMatrix();

/***
 * This fucntion performs a EyeOnHand calibration for a G2 sensor mounted on the
 *Kuka arm
 *
 * @param    system  The Gocator System object
 * @param    sensor  The Gocator Sensor object
 * @param    sensor  A pointer to the KukaRobotDriver object
 * @param    handEye Return parameter of the hand eye calibration matrix
 *
 * @return   Status indicator
 ***/
kStatus Calibrate(GoSystem system, GoSensor sensor, GoRobot::Driver *robotDriver,
                  GoRobot::Matrix *handEye)
{
    std::vector<GoRobot::Matrix> rPoses, sPoses;
    std::vector<GoRobot::Matrix> tcpPoses(NUMBER_OF_POSES);
    std::vector<GoRobot::Matrix> results(tcpPoses.size());
    GoDataSet dataset = kNULL;
    GoRobot::Matrix poseMatrix;

    kTry
    {
        // The function EyeOnHandCalibrationPoses returns a list of poses on a dome
        // looking down on an object in the center. Which is useful for taking
        // multiple scans of the same object (ball bar) from different angles. Note
        // the ballbar should be laying flat at around Z=0
        GoRobot::EyeOnHandCalibrationPoses(tcpPoses.size(), 7, initPose, tcpPoses.data());
        size_t i = 0;
        for (GoRobot::Matrix startPose : tcpPoses) {
            // Find the end pose of the frame
            spdlog::info("Pose {}", i);
            GoRobot::Matrix endPose = startPose * movePose;
            spdlog::info(formatMatrix("Start Pose", startPose));
            spdlog::info(formatMatrix("End Pose", endPose));

            robotDriver->move(&startPose, 1);
            rPoses.push_back(robotDriver->get_flange_pose());
            spdlog::info(formatMatrix("Flange", robotDriver->get_flange_pose()));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            kTest(GoSensor_Start(sensor));
            // Trigger a surface generation on the sensor
            kTest(GoSensor_Trigger(sensor));
            robotDriver->move(&endPose, 1);

            kTest(GoSystem_ReceiveData(system, &dataset, 20000000));
            poseMatrix = GoRobot::GetBallBarPoseMeasurement(dataset, 0);
            sPoses.push_back(poseMatrix);
            spdlog::info(formatMatrix("Sensor", poseMatrix));

            kDestroyRef(&dataset);

            kTest(GoSensor_Stop(sensor));
        }
        robotDriver->move(&initPose);

        // Perform the calibration based on the list of robot poses and the ballbar
        // poses
        *handEye = GoRobot::CalibEyeOnHandSystem(rPoses.data(), sPoses.data(), (k32s)rPoses.size());

        spdlog::info(formatMatrix("Hand Eye Calibration Matrix", *handEye));
    }
    kFinally
    {
        kDestroyRef(&dataset);
        kEndFinally();
    }

    return kOK;
}

/***
 * This fucntion is an application example to locating and moving to an object.
 * It scans the ballbar again, and moves the TCP to be above the center of one
 *of the balls
 *
 * @param    system      The Gocator System object
 * @param    sensor      The Gocator Sensor object
 * @param    sensor      A pointer to the KukaRobotDriver object
 * @param    handEye     The hand eye calibration matrix, obtained from the
 *Calibrate function
 *
 * @return   Status indicator
 ***/
kStatus MoveToBallBar(GoSystem system, GoSensor sensor, GoRobot::Driver *robotDriver,
                      GoRobot::Matrix *handEye)
{
    GoRobot::Matrix endPose = initPose * movePose;
    GoRobot::Matrix flangePose;

    GoRobot::Matrix ballbar_in_sensor;
    GoRobot::Matrix ballbar_in_base;
    GoRobot::Matrix ballbar_in_base_flipped_above;
    GoRobot::Matrix ballbar_in_base_flipped;
    GoDataSet dataset = kNULL;

    kTry
    {
        kTest(GoSensor_Start(sensor));

        robotDriver->move(&initPose);
        flangePose = robotDriver->get_flange_pose();

        // Trigger a surface generation on the sensor
        kTest(GoSensor_Trigger(sensor));
        robotDriver->move(&endPose, 1);

        kTest(GoSystem_ReceiveData(system, &dataset, 20000000));

        // Replace this function with the measurements of the feature you are trying
        // to find
        ballbar_in_sensor = GoRobot::GetBallBarPoseMeasurement(dataset, 0);

        kTest(GoSensor_Stop(sensor));

        // "Locate" will transform the pose from Sensor coordinate system to Base
        // coordinate system
        GoRobot::LocateEyeOnHandSystem(&flangePose, &ballbar_in_sensor, 1, *handEye,
                                       &ballbar_in_base);
        ballbar_in_base_flipped = ballbar_in_base
                                  * GoRobot::KukaPose(0, 0, 0, 180, 0, 180).toMatrix();
        ballbar_in_base_flipped_above = ballbar_in_base_flipped
                                        * GoRobot::KukaPose(0, 0, 20, 0, 0, 0).toMatrix();

        robotDriver->move(&ballbar_in_base_flipped_above);
    }
    kFinally
    {
        kDestroyRef(&dataset);
        kEndFinally();
    }

    return kOK;
}

int main()
{
    // Initialize the logger
    auto logger = spdlog::basic_logger_mt("basic_logger", "log.txt");
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::info);  // Set global log level to info
    spdlog::flush_on(spdlog::level::info);   // Flush log on each info level message

    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    kIpAddress ipAddress;
    GoSetup setup = kNULL;
    GoSurfaceGeneration generation = kNULL;

    GoRobot::Matrix handEye;

    GoRobot::Driver *robotDriver = kNULL;

    kTry
    {
        kTest(GoSdk_Construct(&api));

        robotDriver = (GoRobot::Driver *)new KukaRobotDriver(ROBOT_IP_ADDRESS, ROBOT_PORT,
                                                             kObject_Alloc(api));

        robotDriver->set_base(GoRobot::MATRIX_UNITY);
        robotDriver->set_tcp(tcp);
        robotDriver->set_acc_speed(ACCELERATION, SPEED);

        kTest(GoSystem_Construct(&system, kNULL));
        kTest(kIpAddress_Parse(&ipAddress, SENSOR_IP_ADDRESS));
        kTest(GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor));
        kTest(GoSensor_Connect(sensor));
        kTest(GoSystem_EnableData(system, kTRUE));

        // "SetupCalibrationJob" will create the calibration job on the sensor
        GoRobot::SetupCalibrationJob(sensor);

        // Add to the job file settings to indicate the created surface length
        kTest(kNULL != (setup = GoSensor_Setup(sensor)));
        kTest(GoSetup_SetScanMode(setup, GO_MODE_SURFACE));
        kTest(kNULL != (generation = GoSetup_SurfaceGeneration(setup)));
        kTest(GoSurfaceGenerationFixedLength_SetLength(generation, SURFACE_LENGTH));

        kTest(Calibrate(system, sensor, robotDriver, &handEye));
        kTest(MoveToBallBar(system, sensor, robotDriver, &handEye));
    }
    kFinally
    {
        delete robotDriver;
        kDestroyRef(&system);
        kDestroyRef(&api);
        kEndFinally();
    }
}
